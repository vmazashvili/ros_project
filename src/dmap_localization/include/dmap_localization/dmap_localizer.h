#ifndef DMAP_LOCALIZER_H
#define DMAP_LOCALIZER_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose.h>

class DMapLocalizer
{
public:
    DMapLocalizer();

private:
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void convertMapToPointCloud(const nav_msgs::OccupancyGrid::ConstPtr& map);

    ros::NodeHandle nh_;
    ros::Subscriber map_sub_;
    ros::Subscriber initial_pose_sub_;
    ros::Subscriber scan_sub_;
    ros::Publisher point_cloud_pub_;

    geometry_msgs::Pose initial_pose_;
    bool initial_pose_received_;
};

#endif // DMAP_LOCALIZER_H

