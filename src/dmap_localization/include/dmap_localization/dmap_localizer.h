#ifndef DMAP_LOCALIZER_H
#define DMAP_LOCALIZER_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

class DMapLocalizer
{
public:
    DMapLocalizer();
    void publishSimulatedOdometry();

private:
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void convertMapToPointCloud(const nav_msgs::OccupancyGrid::ConstPtr& map);
    void convertScanToPointCloud(const sensor_msgs::LaserScan::ConstPtr& scan);
    void alignScans();
    void broadcastTransform(const Eigen::Matrix4f& transformation);

    ros::NodeHandle nh_;
    ros::Subscriber map_sub_;
    ros::Subscriber initial_pose_sub_;
    ros::Subscriber scan_sub_;
    ros::Subscriber odom_sub_; // Changed to Subscriber
    ros::Publisher point_cloud_pub_;
    ros::Publisher odom_pub_;
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan_cloud_;

    geometry_msgs::Pose initial_pose_;
    bool initial_pose_received_;
    bool map_received_;
    tf::TransformBroadcaster br_;

    geometry_msgs::Pose current_pose_;
    ros::Time last_odom_time_;
};

#endif // DMAP_LOCALIZER_H

