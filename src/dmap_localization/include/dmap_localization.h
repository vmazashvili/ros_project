#ifndef DMAP_LOCALIZATION_H
#define DMAP_LOCALIZATION_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>

class DMapLocalization
{
public:
    DMapLocalization();
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void computeTransform();

private:
    ros::NodeHandle nh_;
    ros::Subscriber map_sub_;
    ros::Subscriber scan_sub_;
    ros::Subscriber initialpose_sub_;
    ros::Publisher transform_pub_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    nav_msgs::OccupancyGrid::ConstPtr map_;
    sensor_msgs::LaserScan::ConstPtr scan_;
    geometry_msgs::PoseWithCovarianceStamped initial_pose_;
    bool initial_pose_received_;
};

#endif // DMAP_LOCALIZATION_H

