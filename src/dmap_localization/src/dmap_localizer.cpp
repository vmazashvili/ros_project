#include "dmap_localization/dmap_localizer.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Matrix3x3.h>


DMapLocalizer::DMapLocalizer()
: initial_pose_received_(false),
  map_received_(false),
  map_cloud_(new pcl::PointCloud<pcl::PointXYZ>),
  scan_cloud_(new pcl::PointCloud<pcl::PointXYZ>)
{
    map_sub_ = nh_.subscribe("/map", 1, &DMapLocalizer::mapCallback, this);
    initial_pose_sub_ = nh_.subscribe("/initialpose", 1, &DMapLocalizer::initialPoseCallback, this);
    scan_sub_ = nh_.subscribe("/scan", 1, &DMapLocalizer::scanCallback, this);
    point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("aligned_scan_point_cloud", 1);
}

void DMapLocalizer::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    ROS_INFO("Map received");
    convertMapToPointCloud(msg);
    map_received_ = true;
}

void DMapLocalizer::initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    initial_pose_ = msg->pose.pose;
    initial_pose_received_ = true;
    ROS_INFO("Initial pose received: [x: %f, y: %f, z: %f, orientation: (x: %f, y: %f, z: %f, w: %f)]",
             initial_pose_.position.x,
             initial_pose_.position.y,
             initial_pose_.position.z,
             initial_pose_.orientation.x,
             initial_pose_.orientation.y,
             initial_pose_.orientation.z,
             initial_pose_.orientation.w);
}

void DMapLocalizer::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    ROS_INFO("Laser scan received");
    convertScanToPointCloud(msg);
    if (map_received_)
    {
        alignScans();
    }
}

void DMapLocalizer::convertMapToPointCloud(const nav_msgs::OccupancyGrid::ConstPtr& map)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;

    for (unsigned int y = 0; y < map->info.height; y++)
    {
        for (unsigned int x = 0; x < map->info.width; x++)
        {
            unsigned int i = x + y * map->info.width;
            if (map->data[i] > 50)
            {
                pcl::PointXYZ point;
                point.x = x * map->info.resolution + map->info.origin.position.x;
                point.y = y * map->info.resolution + map->info.origin.position.y;
                point.z = 0.0;
                cloud.push_back(point);
            }
        }
    }

    *map_cloud_ = cloud;

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloud, output);
    output.header.frame_id = "map";
    point_cloud_pub_.publish(output);

    ROS_INFO("Map point cloud size: %lu", cloud.size());
}

void DMapLocalizer::convertScanToPointCloud(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;

    for (size_t i = 0; i < scan->ranges.size(); ++i)
    {
        if (std::isfinite(scan->ranges[i]))
        {
            float angle = scan->angle_min + i * scan->angle_increment;
            pcl::PointXYZ point;
            point.x = scan->ranges[i] * cos(angle);
            point.y = scan->ranges[i] * sin(angle);
            point.z = 0.0;
            cloud.push_back(point);
        }
    }

    *scan_cloud_ = cloud;

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloud, output);
    output.header.frame_id = scan->header.frame_id;
    point_cloud_pub_.publish(output);

    ROS_INFO("Scan point cloud size: %lu", cloud.size());
}

void DMapLocalizer::alignScans()
{
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(scan_cloud_);
    icp.setInputTarget(map_cloud_);

    pcl::PointCloud<pcl::PointXYZ> aligned_cloud;
    icp.align(aligned_cloud);

    if (icp.hasConverged())
    {
        Eigen::Matrix4f transformation = icp.getFinalTransformation();
        ROS_INFO_STREAM("ICP converged with fitness score: " << icp.getFitnessScore());
        ROS_INFO_STREAM("Transformation matrix:\n" << transformation);

        // Broadcast the transform
        broadcastTransform(transformation);

        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(aligned_cloud, output);
        output.header.frame_id = "map";
        point_cloud_pub_.publish(output);
    }
    else
    {
        ROS_WARN("ICP did not converge.");
    }
}

void DMapLocalizer::broadcastTransform(const Eigen::Matrix4f& transformation)
{
    tf::Transform tf_transform;
    tf::Matrix3x3 rotation_matrix(
        transformation(0, 0), transformation(0, 1), transformation(0, 2),
        transformation(1, 0), transformation(1, 1), transformation(1, 2),
        transformation(2, 0), transformation(2, 1), transformation(2, 2)
    );

    tf_transform.setBasis(rotation_matrix);
    tf_transform.setOrigin(tf::Vector3(transformation(0, 3), transformation(1, 3), transformation(2, 3)));

    br_.sendTransform(tf::StampedTransform(tf_transform, ros::Time::now(), "map", "odom"));
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "dmap_localizer");
    DMapLocalizer localizer;
    ros::spin();
    return 0;
}

