#ifndef GENERATE_LANE_HELPER_H_
#define GENERATE_LANE_HELPER_H_

#include <Eigen/Core>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

typedef pcl::PointCloud<pcl::PointXYZI> CloudT;
typedef pcl::PointXYZI PointT;

Eigen::Matrix4f convertPose(geometry_msgs::PoseWithCovarianceStamped init);
geometry_msgs::PoseWithCovarianceStamped convertPose(Eigen::Matrix4f pose);
CloudT::Ptr getPiece(CloudT::Ptr input, pcl::PointXYZ origin, double size);
CloudT::Ptr subsample(CloudT::Ptr input, double ratio);
CloudT::Ptr removeNan(CloudT::Ptr input);
double distance(Eigen::Matrix4f m);
CloudT::Ptr removeHighZ(CloudT::Ptr input, double min, double thres);
CloudT::Ptr filterGround(CloudT::Ptr cloud, double thres);
CloudT::Ptr filterGround32(CloudT::Ptr cloud, double thres, int skip);

#endif
