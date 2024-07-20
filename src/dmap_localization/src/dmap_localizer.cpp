#include "dmap_localization/dmap_localizer.h"

DMapLocalizer::DMapLocalizer()
{
    map_sub_ = nh_.subscribe("/map", 1, &DMapLocalizer::mapCallback, this);
    initial_pose_sub_ = nh_.subscribe("/initialpose", 1, &DMapLocalizer::initialPoseCallback, this);
    scan_sub_ = nh_.subscribe("/scan", 1, &DMapLocalizer::scanCallback, this);
}

void DMapLocalizer::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    ROS_INFO("Map received");
}

void DMapLocalizer::initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    ROS_INFO("Initial pose received");
}

void DMapLocalizer::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    ROS_INFO("Laser scan received");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dmap_localizer");
    DMapLocalizer localizer;
    ros::spin();
    return 0;
}

