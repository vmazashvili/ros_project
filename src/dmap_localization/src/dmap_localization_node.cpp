#include "dmap_localization.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

DMapLocalization::DMapLocalization() : tf_listener_(tf_buffer_), initial_pose_received_(false)
{
    map_sub_ = nh_.subscribe("map", 10, &DMapLocalization::mapCallback, this);
    scan_sub_ = nh_.subscribe("scan", 10, &DMapLocalization::scanCallback, this);
    initialpose_sub_ = nh_.subscribe("initialpose", 10, &DMapLocalization::initialPoseCallback, this);
}

void DMapLocalization::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    map_ = msg;
    ROS_INFO("Received map data");
}

void DMapLocalization::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    scan_ = msg;
    ROS_INFO("Received laser scan data");
}

void DMapLocalization::initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    initial_pose_ = *msg;
    initial_pose_received_ = true;
    ROS_INFO("Received initial pose");
}

void DMapLocalization::computeTransform()
{
    if (!map_ || !scan_ || !initial_pose_received_)
        return;

    // Implement ICP to compute the transform between the scan and the map

    geometry_msgs::TransformStamped transform;
    transform.header.stamp = ros::Time::now();
    transform.header.frame_id = "map";
    transform.child_frame_id = "odom";

    // Set transform values (example values)
    transform.transform.translation.x = initial_pose_.pose.pose.position.x;
    transform.transform.translation.y = initial_pose_.pose.pose.position.y;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation = initial_pose_.pose.pose.orientation;

    tf_broadcaster_.sendTransform(transform);
    ROS_INFO("Published transform from map to odom");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dmap_localization");
    DMapLocalization dmap_localization;

    ros::Rate rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        dmap_localization.computeTransform();
        rate.sleep();
    }

    return 0;
}

