#include <chrono>
#include <generate_lane/image.h>

ImageCreator::ImageCreator(ros::NodeHandle nh)
{
  ros::NodeHandle nh_params("~");
  nh_params.param("thres", thres, 0.95);
  nh_params.param("layer", layer_, 32);
  nh_params.param("skip", skip_, 28);

  point_sub_ = nh.subscribe<sensor_msgs::PointCloud2>("input", 10, &ImageCreator::pointCallback, this);
  point_pub_ = nh.advertise<sensor_msgs::PointCloud2>("output", 10);
}

void
ImageCreator::pointCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  CloudT::Ptr cloud(new CloudT());
  pcl::fromROSMsg(*msg, *cloud);
  CloudT::Ptr ground;
  if (layer_ == 32)
  {
    ground = filterGround32(cloud, thres, skip_);
  }
  else
  {
    ground = filterGround(cloud, thres);
  }

  sensor_msgs::PointCloud2 out;
  pcl::toROSMsg(*ground, out);
  out.header = msg->header;
  point_pub_.publish(out);
}

int
main(int argc, char* argv[])
{
  ros::init(argc, argv, "image_creator");
  ros::NodeHandle nh;
  ImageCreator r(nh);
  ros::spin();
  return 0;
}
