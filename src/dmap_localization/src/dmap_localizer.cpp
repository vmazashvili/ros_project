#include "dmap_localization/dmap_localizer.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/icp.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

/*
- DMapLocalizer Constructor initializes the node handle `nh_`
- Sets up subscribers for the laser scan, odometry, map topics
- Initializes the publishers and point clouds. 
  - The node listens to:
    - `map`
    - `scan`
    - `odom`
  - publishes:
    - `localized_pose` (estimated pose)
*/ 
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


/*
- Converts the received occupancy grid map into a point cloud representation `map_cloud_`
- Iterates over the grid cells, checks if the cell is occupied
- Adds the corresponding world coordinates to the `map_cloud_`

- This point cloud is later used in ICP algorithm to alighn the laser scan data with the map
*/
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


/*
- Processes the incoming laser scan data
- Converts is into a point cloud
- Checks if the range data is finite
- Calculates the corresponding (x, y) coordinates in the robot's frame
- Adds them to `scan_cloud`
- Once the scan data is available and `map_cloud_` is not empty, 
  it estimates robot's pose by calling `performLocalization()`
*/
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

/*
- Updates the current odometry data `current_odom_` with the latest poition and orientation
- This data can be used for dead reckoning or as an initial guess for localization
*/
void DMapLocalizer::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  ROS_INFO("Received odometry data with position (%f, %f) and orientation (%f)",
    msg->pose.pose.position.x,
    msg->pose.pose.position.y,
    msg->pose.pose.orientation.z);
  current_odom_ = *msg;
}

/*
- Alligns the `scan_cloud_` laser scan data with `map_cloud_` map data
- Using the Iterative Closest Point (ICP) algorithm
- If the alignment converges, the function:
  - Extracts the transformation matrix
  - Converts it to a `geometry_msgs::PoseStamped` message
  - Publishes the pose
    - Also calls `broadcastTransform` to:
      - Broadcast the transform between the "map" and "base_link" frames
*/
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


/*
- Broadcasts the transfrom from the "map" frame to the "base_link" frame,
  based on the calculated localized pose
- This broadcast allows other nodes to use the transform data for various tasks, 
  such as visualizing the robot's position in RViz
*/
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

/*
- Initializes the ROS node and the instance of `DMapLocalizer`
- Calls `ros::spin()` to keep the node running, lietening for incoming
  messages and calling the appropriate callbacks
*/
int main(int argc, char** argv)
{
  ros::init(argc, argv, "dmap_localizer");
  DMapLocalizer localizer;
  ros::spin();

  return 0;
}
