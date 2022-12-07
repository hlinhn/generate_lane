#ifndef GENERATE_LANE_GPS_INIT_H_
#define GENERATE_LANE_GPS_INIT_H_

#include <generate_lane/helper.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

class GpsInit
{
private:
  ros::Publisher align_pub_;
  ros::Publisher map_pub_;
  ros::Publisher pose_pub_;
  ros::Subscriber cloud_sub_;
  ros::Subscriber init_sub_;

  void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
  void poseCallback(geometry_msgs::PoseWithCovarianceStamped msg);
  CloudT::Ptr last_cloud_;
  CloudT::Ptr full_cloud_;
  double size_;
  pcl::IterativeClosestPoint<PointT, PointT> icp_;

public:
  GpsInit(ros::NodeHandle nh);
  ~GpsInit() {}
};

#endif
