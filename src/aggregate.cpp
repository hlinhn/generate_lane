#include <chrono>
#include <generate_lane/aggregate.h>
#include <iostream>
#include <pcl_conversions/pcl_conversions.h>

Aggregate::Aggregate(ros::NodeHandle nh)
{
  ros::NodeHandle nh_params("~");

  nh_params.param("frame_id", frame_id_, std::string("odom"));
  nh_params.param("factor", factor_, 7.0);
  nh_params.param("half_size", half_size, 30.0);
  nh_params.param("thres_lane", thres_lane_, 100.0);
  nh_params.param("min_point", min_point_, 20);
  nh_params.param("resolution", resolution_, 0.05);
  nh_params.param("distance", min_dist_, 0.07);

  cloud2_sub_.subscribe(nh, "input", 1);
  tf_filter_ = new tf::MessageFilter<sensor_msgs::PointCloud2>(cloud2_sub_, tf_, frame_id_, 10);
  tf_filter_->registerCallback(boost::bind(&Aggregate::pointCallback, this, _1));
  lane_pub_ = nh.advertise<sensor_msgs::PointCloud2>("output_curb", 10);
  first_ = false;
}

CloudT::Ptr
Aggregate::pickLane(CloudT::Ptr points, double resolution, cv::Point2d current_pos)
{
  cv::Point2d minpt(current_pos.x - half_size, current_pos.y - half_size);
  cv::Point2d maxpt(current_pos.x + half_size, current_pos.y + half_size);

  unsigned int sizex = std::ceil((maxpt.x - minpt.x) / resolution);
  unsigned int sizey = std::ceil((maxpt.y - minpt.y) / resolution);

  cv::Mat sumIm(sizex, sizey, CV_64FC1, cv::Scalar(0));
  cv::Mat countIm(sizex, sizey, CV_8UC1, cv::Scalar(0));

  for (const auto p : *points)
  {
    int x = std::ceil((p.x - minpt.x) / resolution);
    int y = std::ceil((p.y - minpt.y) / resolution);
    if (x < 0 || x >= sizex || y < 0 || y >= sizey)
      continue;

    sumIm.at<double>(x, y) += p.intensity;
    countIm.at<unsigned char>(x, y) += 1;
  }

  cv::Mat bindiffIm(sizex, sizey, CV_64FC1, cv::Scalar(0));

  for (unsigned int i = 0; i < countIm.rows; i++)
  {
    for (unsigned int j = 0; j < countIm.cols; j++)
    {
      if (countIm.at<unsigned char>(i, j) < min_point_)
        continue;
      double d = sumIm.at<double>(i, j) / countIm.at<unsigned char>(i, j);
      bindiffIm.at<double>(i, j) = d; // static_cast<unsigned char>(d);
    }
  }

  CloudT::Ptr curb(new CloudT());
  for (const auto p : *points)
  {
    int x = std::ceil((p.x - minpt.x) / resolution);
    int y = std::ceil((p.y - minpt.y) / resolution);
    if (x < 0 || x >= sizex || y < 0 || y >= sizey)
      continue;
    if (bindiffIm.at<double>(x, y) > thres_lane_)
      curb->points.push_back(p);
  }

  return curb;
}

CloudT::Ptr
Aggregate::addCloud(CloudT::Ptr cloud, CloudT::Ptr sum)
{
  *sum += *cloud;
  if (sum->points.size() > cloud->points.size() * factor_)
  {
    sum->erase(sum->points.begin(), sum->points.begin() + cloud->points.size());
  }
  return sum;
}

CloudT::Ptr
Aggregate::process(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  CloudT::Ptr cloud(new CloudT());
  pcl::fromROSMsg(*msg, *cloud);

  CloudT::Ptr transformed(new CloudT());
  pcl_ros::transformPointCloud<PointT>(frame_id_, *cloud, *transformed, tf_);
  CloudT::Ptr filtered(new CloudT());
  filtered = removeNan(transformed);
  // CloudT::Ptr cut = removeHighZ(filtered, -1.0, 2.0);
  return filtered;
}

void
Aggregate::pointCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  tf::StampedTransform to_base;
  try
  {
    tf_.lookupTransform(frame_id_, "base_link", msg->header.stamp, to_base);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    return;
  }
  Eigen::Matrix4f trans_mat;
  pcl_ros::transformAsMatrix(to_base, trans_mat);

  if (!first_)
  {
    first_ = true;
    last_saved_ = trans_mat;
    cut_above_cloud_ = process(msg);
    return;
  }

  Eigen::Matrix4f diff = last_saved_.inverse() * trans_mat;
  if (distance(diff) < min_dist_)
    return;

  last_saved_ = trans_mat;

  CloudT::Ptr processed = process(msg);
  cut_above_cloud_ = addCloud(processed, cut_above_cloud_);
  cv::Point2d current_pos(trans_mat(0, 3), trans_mat(1, 3));
  CloudT::Ptr lane = pickLane(cut_above_cloud_, resolution_, current_pos);

  sensor_msgs::PointCloud2 lane_msg;
  pcl::transformPointCloud(*lane, *lane, trans_mat.inverse());
  pcl::toROSMsg(*lane, lane_msg);
  lane_msg.header = msg->header;
  lane_msg.header.frame_id = "base_link";
  lane_pub_.publish(lane_msg);
}

int
main(int argc, char* argv[])
{
  ros::init(argc, argv, "aggregate");
  ros::NodeHandle nh;
  Aggregate r(nh);
  ros::spin();
  return 0;
}
