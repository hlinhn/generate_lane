#include <chrono>
#include <generate_lane/gps_init.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

GpsInit::GpsInit(ros::NodeHandle nh)
{
  ros::NodeHandle nh_params("~");
  std::string big_cloud;
  nh_params.param("cloud", big_cloud, std::string(""));
  nh_params.param("size", size_, 40.0);
  CloudT::Ptr cloud(new CloudT());
  pcl::io::loadPCDFile(big_cloud, *cloud);
  full_cloud_ = cloud;
  init_sub_ =
      nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("initialpose_gps", 1, &GpsInit::poseCallback, this);
  cloud_sub_ = nh.subscribe<sensor_msgs::PointCloud2>("output_curb", 10, &GpsInit::cloudCallback, this);
  map_pub_ = nh.advertise<sensor_msgs::PointCloud2>("map", 1, true);
  align_pub_ = nh.advertise<sensor_msgs::PointCloud2>("aligned", 1);
  pose_pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose_mcl", 1);

  sensor_msgs::PointCloud2 out;
  pcl::toROSMsg(*full_cloud_, out);
  out.header.stamp = ros::Time::now();
  out.header.frame_id = "map";
  map_pub_.publish(out);

  icp_.setMaximumIterations(20);
  icp_.setMaxCorrespondenceDistance(3.0);
}

void
GpsInit::poseCallback(geometry_msgs::PoseWithCovarianceStamped msg)
{
  Eigen::Matrix4f last_pos = convertPose(msg);
  pcl::PointXYZ p(last_pos(0, 3), last_pos(1, 3), 0);
  CloudT::Ptr piece = getPiece(full_cloud_, p, size_);
  icp_.setInputTarget(piece);

  CloudT::Ptr cloud(new CloudT());
  pcl::transformPointCloud(*last_cloud_, *cloud, last_pos);

  icp_.setInputSource(cloud);
  CloudT::Ptr aligned(new CloudT());
  icp_.align(*aligned);

  sensor_msgs::PointCloud2 out_align;
  pcl::toROSMsg(*aligned, out_align);
  out_align.header = msg.header;
  out_align.header.frame_id = "map";
  align_pub_.publish(out_align);

  Eigen::Matrix4f transformation = icp_.getFinalTransformation() * last_pos;
  auto corrected = convertPose(transformation);
  pose_pub_.publish(corrected);
}

void
GpsInit::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  CloudT::Ptr cloud(new CloudT());
  pcl::fromROSMsg(*msg, *cloud);

  last_cloud_ = cloud;
}

int
main(int argc, char* argv[])
{
  ros::init(argc, argv, "gps_init");
  ros::NodeHandle nh;
  GpsInit r(nh);
  ros::spin();
  return 0;
}
