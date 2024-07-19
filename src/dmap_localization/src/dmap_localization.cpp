#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/LaserScan.h>

class DMapLocalizer
{
public:
    DMapLocalizer(){
        map_sub_ = nh_.subs#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/LaserScan.h>

class DMapLocalizer
{
public:
    DMapLocalizer()
    {
        map_sub_ = nh_.subscribe("/map", 1, &DMapLocalizer::mapCallback, this);
        initial_pose_sub_ = nh_.subscribe("/initialpose", 1, &DMapLocalizer::initialPoseCallback, this);
        scan_sub_ = nh_.subscribe("/scan", 1, &DMapLocalizer::scanCallback, this);
    }

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
    {
        ROS_INFO("Map received");
    }

    void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
    {
        ROS_INFO("Initial pose received");
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
        ROS_INFO("Laser scan received");
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber map_sub_;
    ros::Subscriber initial_pose_sub_;
    ros::Subscriber scan_sub_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dmap_localizer");
    DMapLocalizer localizer;
    ros::spin();
    return 0;
}


}
