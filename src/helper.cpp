#include <generate_lane/helper.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/random_sample.h>
#include <pcl_conversions/pcl_conversions.h>

Eigen::Matrix4f
convertPose(geometry_msgs::PoseWithCovarianceStamped init)
{
  Eigen::Matrix4f last_pos = Eigen::Matrix4f::Identity();
  last_pos(0, 3) = init.pose.pose.position.x;
  last_pos(1, 3) = init.pose.pose.position.y;
  last_pos(2, 3) = init.pose.pose.position.z;

  Eigen::Quaternionf q(init.pose.pose.orientation.w,
                       init.pose.pose.orientation.x,
                       init.pose.pose.orientation.y,
                       init.pose.pose.orientation.z);

  last_pos.block<3, 3>(0, 0) = q.matrix();
  return last_pos;
}

geometry_msgs::PoseWithCovarianceStamped
convertPose(Eigen::Matrix4f pose)
{
  geometry_msgs::PoseWithCovarianceStamped msg;
  msg.pose.pose.position.x = pose(0, 3);
  msg.pose.pose.position.y = pose(1, 3);
  msg.pose.pose.position.z = pose(2, 3);

  Eigen::Matrix3f r = pose.block<3, 3>(0, 0);
  Eigen::Quaternionf q(r);

  msg.pose.pose.orientation.x = q.x();
  msg.pose.pose.orientation.y = q.y();
  msg.pose.pose.orientation.z = q.z();
  msg.pose.pose.orientation.w = q.w();

  return msg;
}

CloudT::Ptr
getPiece(CloudT::Ptr input, pcl::PointXYZ origin, double size)
{
  CloudT::Ptr filtered(new CloudT());
  pcl::ConditionalRemoval<PointT> condrem;
  pcl::ConditionAnd<PointT>::Ptr cond(new pcl::ConditionAnd<PointT>());
  cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr(
      new pcl::FieldComparison<PointT>("x", pcl::ComparisonOps::GT, origin.x - size)));
  cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr(
      new pcl::FieldComparison<PointT>("x", pcl::ComparisonOps::LT, origin.x + size)));
  cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr(
      new pcl::FieldComparison<PointT>("y", pcl::ComparisonOps::GT, origin.y - size)));
  cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr(
      new pcl::FieldComparison<PointT>("y", pcl::ComparisonOps::LT, origin.y + size)));

  condrem.setCondition(cond);
  condrem.setInputCloud(input);
  condrem.setKeepOrganized(true);
  condrem.filter(*filtered);
  return filtered;
}

CloudT::Ptr
subsample(CloudT::Ptr input, double ratio)
{
  CloudT::Ptr subsampled(new CloudT());
  pcl::RandomSample<PointT> extract;
  extract.setInputCloud(input);
  extract.setSample(int(input->size() * ratio));
  extract.filter(*subsampled);
  return subsampled;
}

CloudT::Ptr
removeNan(CloudT::Ptr input)
{
  CloudT::Ptr output(new CloudT());
  for (const auto p : *input)
  {
    if (std::isfinite(p.x))
    {
      output->points.push_back(p);
    }
  }
  return output;
}

double
distance(Eigen::Matrix4f m)
{
  return sqrt(m(0, 3) * m(0, 3) + m(1, 3) * m(1, 3) + m(2, 3) * m(2, 3));
}

CloudT::Ptr
removeHighZ(CloudT::Ptr input, double min, double thres)
{
  CloudT::Ptr filtered(new CloudT());
  pcl::ConditionalRemoval<PointT> condrem;
  pcl::ConditionAnd<PointT>::Ptr cond(new pcl::ConditionAnd<PointT>());
  cond->addComparison(
      pcl::FieldComparison<PointT>::ConstPtr(new pcl::FieldComparison<PointT>("z", pcl::ComparisonOps::GT, min)));
  cond->addComparison(
      pcl::FieldComparison<PointT>::ConstPtr(new pcl::FieldComparison<PointT>("z", pcl::ComparisonOps::LT, thres)));

  condrem.setCondition(cond);
  condrem.setInputCloud(input);
  condrem.setKeepOrganized(true);
  condrem.filter(*filtered);
  return filtered;
}

bool
valid(PointT p)
{
  return (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z));
}

double
dotprod(PointT p, PointT q)
{
  return (p.x * q.x + p.y * q.y + p.z * q.z);
}

PointT
sub(PointT p, PointT q)
{
  PointT res;
  res.x = p.x - q.x;
  res.y = p.y - q.y;
  res.z = p.z - q.z;
  return res;
}

PointT
norm(PointT p)
{
  double mag = sqrt(dotprod(p, p));
  PointT res;
  res.x = p.x / mag;
  res.y = p.y / mag;
  res.z = p.z / mag;
  return res;
}

// assume ordered ring
CloudT::Ptr
filterGround(CloudT::Ptr cloud, double thres)
{
  CloudT::Ptr ground(new CloudT());
  pcl::PointIndices::Ptr indices(new pcl::PointIndices());
  std::vector<PointT> base_points;
  std::vector<int> base_ring;
  for (int j = 0; j < cloud->width; j++)
  {
    PointT pnan;
    pnan.x = NAN;
    base_points.push_back(pnan);
    base_ring.push_back(-1);
    for (int i = 0; i < cloud->height; i++)
    {
      PointT p = cloud->at(j, i);
      if (valid(p) && sqrt(dotprod(p, p)) > 1.0)
      {
        base_points[j] = p;
        base_ring[j] = i;
        ground->push_back(p);
        indices->indices.push_back(i * cloud->width + j);
        break;
      }
      else
      {
        continue;
      }
    }
  }

  for (int j = 0; j < cloud->width; j++)
  {
    if (base_ring[j] < 0)
      continue;
    for (int i = base_ring[j] + 2; i < cloud->height; i++)
    {
      PointT p = cloud->at(j, i);
      PointT prev = cloud->at(j, i - 1);
      if (!valid(p) || !valid(prev))
        break;
      PointT pprev = base_points[j];
      double dot = dotprod(norm(sub(pprev, prev)), norm(sub(prev, p)));
      if (dot > thres)
      {
        ground->push_back(p);
        indices->indices.push_back(i * cloud->width + j);
        if (i == base_ring[j] + 2)
        {
          ground->push_back(prev);
          indices->indices.push_back((i - 1) * cloud->width + j);
        }
      }
      else
      {
        break;
      }
    }
  }

  pcl::ExtractIndices<PointT> extract;
  CloudT::Ptr without_ground(new CloudT());
  extract.setInputCloud(cloud);
  extract.setIndices(indices);
  extract.setNegative(false);
  extract.setKeepOrganized(true);
  extract.filter(*without_ground);

  return without_ground;
}

// ring not exactly in order
CloudT::Ptr
filterGround32(CloudT::Ptr cloud, double thres, int skip)
{
  CloudT::Ptr ground(new CloudT());
  pcl::PointIndices::Ptr indices(new pcl::PointIndices());
  std::vector<PointT> base_points;
  std::vector<int> base_ring;
  for (int j = 0; j < cloud->width; j++)
  {
    PointT pnan;
    pnan.x = NAN;
    base_points.push_back(pnan);
    base_ring.push_back(-1);
    for (int i = cloud->height - 1; i >= 0; i--)
    {
      PointT p = cloud->at(j, i);
      if (valid(p) && sqrt(dotprod(p, p)) > 1.0)
      {
        base_points[j] = p;
        base_ring[j] = i;
        break;
      }
      else
      {
        continue;
      }
    }
  }

  for (int j = 0; j < cloud->width; j++)
  {
    if (base_ring[j] < 0)
      continue;
    for (int i = base_ring[j] - 2; i > 0; i--)
    {
      PointT p = cloud->at(j, i);
      PointT prev = cloud->at(j, i + 1);
      if (!valid(p) || !valid(prev))
        break;
      PointT pprev = base_points[j];
      double dot = dotprod(norm(sub(pprev, prev)), norm(sub(prev, p)));
      if (dot > thres)
      {
        if (i >= skip)
          continue;
        ground->push_back(p);
        indices->indices.push_back(i * cloud->width + j);
        if (i == base_ring[j] - 2)
        {
          ground->push_back(prev);
          indices->indices.push_back((i + 1) * cloud->width + j);
        }
      }
      else
      {
        break;
      }
    }
  }

  pcl::ExtractIndices<PointT> extract;
  CloudT::Ptr without_ground(new CloudT());
  extract.setInputCloud(cloud);
  extract.setIndices(indices);
  extract.setNegative(false);
  extract.setKeepOrganized(true);
  extract.filter(*without_ground);

  return without_ground;
}
