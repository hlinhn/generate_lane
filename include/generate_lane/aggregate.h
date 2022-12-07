#ifndef GENERATE_LANE_AGGREGATE_H_
#define GENERATE_LANE_AGGREGATE_H_

#include <generate_lane/helper.h>
#include <message_filters/subscriber.h>
#include <opencv2/opencv.hpp>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/message_filter.h>
#include <tf/transform_listener.h>

class Aggregate
{
private:
  bool first_;
  double min_dist_;
  double half_size;
  double thres_lane_;
  double resolution_;
  double factor_;
  int min_point_;
  std::string frame_id_;

  CloudT::Ptr cut_above_cloud_;
  Eigen::Matrix4f last_saved_;

  ros::Publisher lane_pub_;
  tf::TransformListener tf_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud2_sub_;
  tf::MessageFilter<sensor_msgs::PointCloud2>* tf_filter_;

  void pointCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
  CloudT::Ptr pickLane(CloudT::Ptr points, double resolution, cv::Point2d current_pos);
  CloudT::Ptr addCloud(CloudT::Ptr cloud, CloudT::Ptr sum);
  CloudT::Ptr process(const sensor_msgs::PointCloud2::ConstPtr& msg);

public:
  Aggregate(ros::NodeHandle nh);
  ~Aggregate() {}
};

#endif
