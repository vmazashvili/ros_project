#include "dmap_localization/dmap_localizer.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

DMapLocalizer::DMapLocalizer()
{
    map_sub_ = nh_.subscribe("/map", 1, &DMapLocalizer::mapCallback, this);
    initial_pose_sub_ = nh_.subscribe("/initialpose", 1, &DMapLocalizer::initialPoseCallback, this);
    scan_sub_ = nh_.subscribe("/scan", 1, &DMapLocalizer::scanCallback, this);
    point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("map_point_cloud", 1);
}

void DMapLocalizer::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    ROS_INFO("Map received");
    convertMapToPointCloud(msg);
}

void DMapLocalizer::initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    ROS_INFO("Initial pose received");
}

void DMapLocalizer::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    ROS_INFO("Laser scan received");
}

void DMapLocalizer::convertMapToPointCloud(const nav_msgs::OccupancyGrid::ConstPtr& map)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;

    // Iterate through the occupancy grid and convert occupied cells to points
    for (unsigned int y = 0; y < map->info.height; y++)
    {
        for (unsigned int x = 0; x < map->info.width; x++)
        {
            unsigned int i = x + y * map->info.width;
            if (map->data[i] > 50)  // Threshold to consider cell as occupied
            {
                pcl::PointXYZ point;
                point.x = x * map->info.resolution + map->info.origin.position.x;
                point.y = y * map->info.resolution + map->info.origin.position.y;
                point.z = 0.0;
                cloud.push_back(point);
            }
        }
    }

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloud, output);
    output.header.frame_id = "map";
    point_cloud_pub_.publish(output);

    ROS_INFO("Point cloud size: %lu", cloud.size());
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dmap_localizer");
    DMapLocalizer localizer;
    ros::spin();
    return 0;
}

