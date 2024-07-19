#include "dmap_localization/dmap_localizer.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <nav_msgs/Odometry.h>

DMapLocalizer::DMapLocalizer()
: initial_pose_received_(false),
  map_received_(false),
  map_cloud_(new pcl::PointCloud<pcl::PointXYZ>),
  scan_cloud_(new pcl::PointCloud<pcl::PointXYZ>)
{
    odom_sub_ = nh_.subscribe("/odom", 1, &DMapLocalizer::odomCallback, this);
    map_sub_ = nh_.subscribe("/map", 1, &DMapLocalizer::mapCallback, this);
    initial_pose_sub_ = nh_.subscribe("/initialpose", 1, &DMapLocalizer::initialPoseCallback, this);
    scan_sub_ = nh_.subscribe("/scan", 1, &DMapLocalizer::scanCallback, this);
    point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("aligned_scan_point_cloud", 1);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom_transformed", 50); // Correct topic name
    
    last_odom_time_ = ros::Time::now();
    current_pose_.position.x = 0.0;
    current_pose_.position.y = 0.0;
    current_pose_.position.z = 0.0;
    current_pose_.orientation.w = 1.0;
    current_pose_.orientation.x = 0.0;
    current_pose_.orientation.y = 0.0;
    current_pose_.orientation.z = 0.0;
}

void DMapLocalizer::publishSimulatedOdometry()
{
    ros::Time current_time = ros::Time::now();
    double dt = (current_time - last_odom_time_).toSec();

    // Simple constant velocity model
    double vx = 0.05; // m/s
    double vy = 0.025; // m/s
    double vth = 0.005; // rad/s

    current_pose_.position.x += vx * dt;
    current_pose_.position.y += vy * dt;

    tf::Quaternion q;
    tf::quaternionMsgToTF(current_pose_.orientation, q);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    yaw += vth * dt;
    q.setRPY(roll, pitch, yaw);
    tf::quaternionTFToMsg(q, current_pose_.orientation);

    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose = current_pose_;

    // Set some arbitrary covariance
    odom.pose.covariance[0] = 0.1;
    odom.pose.covariance[7] = 0.1;
    odom.pose.covariance[35] = 0.1;

    odom_pub_.publish(odom);
    last_odom_time_ = current_time;
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
    ROS_INFO("Received laser scan with %zu ranges", msg->ranges.size());
    ROS_INFO("Angle min: %f, Angle max: %f, Angle increment: %f", 
             msg->angle_min, msg->angle_max, msg->angle_increment);
    ROS_INFO("Range min: %f, Range max: %f", msg->range_min, msg->range_max);

    // Log the actual range values
    std::stringstream ss;
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
        ss << msg->ranges[i] << " ";
    }
    ROS_INFO_STREAM("Ranges: " << ss.str());

    convertScanToPointCloud(msg);
    ROS_INFO("Converted scan cloud size: %zu", scan_cloud_->size());

    if (scan_cloud_->size() < 10) {
        ROS_WARN("Not enough valid points in scan. Skipping alignment.");
        return;
    }

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
    int valid_points = 0;

    for (size_t i = 0; i < scan->ranges.size(); ++i)
    {
        if (std::isfinite(scan->ranges[i]) && scan->ranges[i] >= scan->range_min && scan->ranges[i] <= scan->range_max)
        {
            float angle = scan->angle_min + i * scan->angle_increment;
            pcl::PointXYZ point;
            point.x = scan->ranges[i] * cos(angle);
            point.y = scan->ranges[i] * sin(angle);
            point.z = 0.0;
            cloud.push_back(point);
            valid_points++;
        }
    }

    *scan_cloud_ = cloud;

    ROS_INFO("Converted %d valid points out of %zu total ranges", valid_points, scan->ranges.size());

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloud, output);
    output.header.frame_id = scan->header.frame_id;
    point_cloud_pub_.publish(output);
}

void DMapLocalizer::alignScans()
{
    if (scan_cloud_->empty() || map_cloud_->empty()) {
        ROS_WARN("Empty point cloud, skipping alignment");
        return;
    }

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(scan_cloud_);
    icp.setInputTarget(map_cloud_);

    // Relaxed ICP parameters
    icp.setMaximumIterations(100);  // Increased from 50
    icp.setTransformationEpsilon(1e-6);  // Relaxed from 1e-8
    icp.setMaxCorrespondenceDistance(0.5);  // Increased from 0.05
    icp.setRANSACOutlierRejectionThreshold(0.05);  // Added RANSAC outlier rejection

    // Create initial guess transform
    Eigen::Affine3f initial_guess = Eigen::Affine3f::Identity();
    initial_guess.translation() << current_pose_.position.x, current_pose_.position.y, current_pose_.position.z;
    Eigen::Quaternionf q(current_pose_.orientation.w, current_pose_.orientation.x, current_pose_.orientation.y, current_pose_.orientation.z);
    initial_guess.rotate(q);

    ROS_INFO_STREAM("Aligning scan cloud (size: " << scan_cloud_->size()
        << ") with map cloud (size: " << map_cloud_->size() << ")");

    pcl::PointCloud<pcl::PointXYZ> aligned_cloud;
    icp.align(aligned_cloud, initial_guess.matrix());

    ROS_INFO_STREAM("ICP finished. Converged: " << (icp.hasConverged() ? "Yes" : "No")
        << ", Score: " << icp.getFitnessScore()
        << ", Num iterations: " << icp.getMaximumIterations());

    if (icp.hasConverged())
    {
        Eigen::Matrix4f transformation = icp.getFinalTransformation();
        double fitness_score = icp.getFitnessScore();

        ROS_INFO_STREAM("ICP converged with fitness score: " << fitness_score);
        ROS_INFO_STREAM("Transformation matrix:\n" << transformation);

        // Update current_pose_
        Eigen::Affine3f final_transform(transformation);
        Eigen::Vector3f translation = final_transform.translation();
        Eigen::Quaternionf rotation(final_transform.rotation());

        current_pose_.position.x = translation.x();
        current_pose_.position.y = translation.y();
        current_pose_.position.z = translation.z();
        current_pose_.orientation.x = rotation.x();
        current_pose_.orientation.y = rotation.y();
        current_pose_.orientation.z = rotation.z();
        current_pose_.orientation.w = rotation.w();

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
    tf::Matrix3x3 tf_rotation(
        transformation(0,0), transformation(0,1), transformation(0,2),
        transformation(1,0), transformation(1,1), transformation(1,2),
        transformation(2,0), transformation(2,1), transformation(2,2)
    );
    tf::Vector3 tf_translation(transformation(0,3), transformation(1,3), transformation(2,3));

    tf_transform.setOrigin(tf_translation);
    tf_transform.setBasis(tf_rotation);

    br_.sendTransform(tf::StampedTransform(tf_transform, ros::Time::now(), "map", "odom"));
    ROS_INFO("Broadcasted transform from 'map' to 'odom'");
}

void DMapLocalizer::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    current_pose_ = msg->pose.pose;
    last_odom_time_ = msg->header.stamp;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dmap_localizer");
    DMapLocalizer localizer;
    ros::Rate rate(10); // 10 Hz
    while (ros::ok())
    {
        localizer.publishSimulatedOdometry();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}


