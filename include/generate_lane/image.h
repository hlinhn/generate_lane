#ifndef GENERATE_LANE_IMAGE_H_
#define GENERATE_LANE_IMAGE_H_

#include <generate_lane/helper.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

class ImageCreator
{
private:
  double thres;
  int layer_;
  int skip_;

  ros::Publisher point_pub_;
  ros::Subscriber point_sub_;
  void pointCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

public:
  ImageCreator(ros::NodeHandle nh);
  ~ImageCreator() {}
};

#endif
