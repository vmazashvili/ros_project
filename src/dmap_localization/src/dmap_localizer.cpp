#include "dmap_localization/dmap_localizer.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/icp.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

DMapLocalizer::DMapLocalizer()
  : nh_("~"), tf_listener_(tf_buffer_)
{
  scan_sub_ = nh_.subscribe("scan", 1, &DMapLocalizer::scanCallback, this);
  odom_sub_ = nh_.subscribe("odom", 1, &DMapLocalizer::odomCallback, this);
  map_sub_ = nh_.subscribe("map", 1, &DMapLocalizer::mapCallback, this);
  
  localized_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("localized_pose", 1);

  map_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  scan_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
}

void DMapLocalizer::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  ROS_INFO("Received map");
  map_cloud_->clear();
  for (unsigned int y = 0; y < msg->info.height; ++y)
  {
    for (unsigned int x = 0; x < msg->info.width; ++x)
    {
      if (msg->data[y * msg->info.width + x] > 50)
      {
        pcl::PointXYZ point;
        point.x = x * msg->info.resolution + msg->info.origin.position.x;
        point.y = y * msg->info.resolution + msg->info.origin.position.y;
        point.z = 0;
        map_cloud_->points.push_back(point);
      }
    }
  }
  map_cloud_->width = map_cloud_->points.size();
  map_cloud_->height = 1;
  map_cloud_->is_dense = true;
}

void DMapLocalizer::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  ROS_INFO("Received a laser scan with %ld ranges", msg->ranges.size());
  scan_cloud_->clear();
  for (size_t i = 0; i < msg->ranges.size(); ++i)
  {
    if (std::isfinite(msg->ranges[i]))
    {
      float angle = msg->angle_min + i * msg->angle_increment;
      pcl::PointXYZ point;
      point.x = msg->ranges[i] * cos(angle);
      point.y = msg->ranges[i] * sin(angle);
      point.z = 0;
      scan_cloud_->points.push_back(point);
    }
  }
  scan_cloud_->width = scan_cloud_->points.size();
  scan_cloud_->height = 1;
  scan_cloud_->is_dense = true;

  if (!map_cloud_->empty())
  {
    performLocalization();
  }
}

void DMapLocalizer::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  ROS_INFO("Received odometry data with position (%f, %f) and orientation (%f)",
    msg->pose.pose.position.x,
    msg->pose.pose.position.y,
    msg->pose.pose.orientation.z);
  current_odom_ = *msg;
}

void DMapLocalizer::performLocalization()
{
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(scan_cloud_);
  icp.setInputTarget(map_cloud_);
  
  pcl::PointCloud<pcl::PointXYZ> aligned_cloud;
  icp.align(aligned_cloud);

  if (icp.hasConverged())
  {
    Eigen::Matrix4f transformation = icp.getFinalTransformation();
    
    geometry_msgs::PoseStamped localized_pose;
    localized_pose.header.stamp = ros::Time::now();
    localized_pose.header.frame_id = "map";
    
    localized_pose.pose.position.x = transformation(0, 3);
    localized_pose.pose.position.y = transformation(1, 3);
    localized_pose.pose.position.z = 0;
    
    Eigen::Quaternionf q(Eigen::Matrix3f(transformation.block<3,3>(0,0)));
    localized_pose.pose.orientation.x = q.x();
    localized_pose.pose.orientation.y = q.y();
    localized_pose.pose.orientation.z = q.z();
    localized_pose.pose.orientation.w = q.w();
    
    localized_pose_pub_.publish(localized_pose);
    
    broadcastTransform(localized_pose);
  }
  else
  {
    ROS_WARN("ICP did not converge");
  }
}

void DMapLocalizer::broadcastTransform(const geometry_msgs::PoseStamped& pose)
{
  static tf2_ros::TransformBroadcaster br;
  
  geometry_msgs::TransformStamped transformStamped;
  
  transformStamped.header.stamp = pose.header.stamp;
  transformStamped.header.frame_id = "map";
  transformStamped.child_frame_id = "base_link";
  
  transformStamped.transform.translation.x = pose.pose.position.x;
  transformStamped.transform.translation.y = pose.pose.position.y;
  transformStamped.transform.translation.z = pose.pose.position.z;
  transformStamped.transform.rotation = pose.pose.orientation;
  
  br.sendTransform(transformStamped);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dmap_localizer");
  ros::NodeHandle nh;

  // Subscribe to laser scan and odometry topics
  //ros::Subscriber laser_sub = nh.subscribe("/scan", 10, scanCallback);
  //ros::Subscriber odom_sub = nh.subscribe("/odom", 10, odomCallback)
  
  DMapLocalizer localizer;
  ros::spin();

  return 0;
}
