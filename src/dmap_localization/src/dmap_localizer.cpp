#include "dmap_localization/dmap_localizer.h"
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <vector>
#include <limits>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <visualization_msgs/Marker.h>
#include <tf2/utils.h>

DMAPLocalizer::DMAPLocalizer() : nh_("~"), tf_listener_(tf_buffer_) {
    initializeROS();
    initializePCL();
}

void DMAPLocalizer::run() {
    ros::spin();
}

void DMAPLocalizer::initializeROS() {
    pose_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("current_pose_marker", 1);
    map_sub_ = nh_.subscribe("/map", 1, &DMAPLocalizer::mapCallback, this);
    scan_sub_ = nh_.subscribe("/scan", 1, &DMAPLocalizer::scanCallback, this);
    initial_pose_sub_ = nh_.subscribe("/initialpose", 1, &DMAPLocalizer::initialPoseCallback, this);
    //step_icp_sub_ = nh_.subscribe("/step_icp", 1, &DMAPLocalizer::stepICP, this);
    
    localized_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("localized_pose", 1, this);
    dmap_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("dmap_cloud", 1, this);
    scan_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("scan_cloud", 1, this);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>();

    // Load parameters
    nh_.param("dmap_threshold", dmap_threshold_, 0.8);
    nh_.param("icp_max_iterations", icp_max_iterations_, 50);
    nh_.param("icp_transformation_epsilon", icp_transformation_epsilon_, 1e-8);
    nh_.param("icp_max_correspondence_distance", icp_max_correspondence_distance_, 1.0);
    icp_timer_ = nh_.createTimer(ros::Duration(1.0), &DMAPLocalizer::stepICP, this);
    icp_timer_.stop();  // Initially stop the timer
}

void DMAPLocalizer::initializePCL() {
    map_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    dmap_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    scan_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
}

void DMAPLocalizer::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    ROS_INFO("Received map");
    map_ = *msg;
    createMapCloud();
    createDMAP();
    publishDMAPCloud();
    icp_timer_.start();  // Start the timer after creating DMAP
}

void DMAPLocalizer::createMapCloud() {
    map_cloud_->clear();
    for (int y = 0; y < map_.info.height; ++y) {
        for (int x = 0; x < map_.info.width; ++x) {
            if (map_.data[y * map_.info.width + x] > 50) {
                pcl::PointXYZ point;
                point.x = x * map_.info.resolution + map_.info.origin.position.x;
                point.y = y * map_.info.resolution + map_.info.origin.position.y;
                point.z = 0;
                map_cloud_->push_back(point);
            }
        }
    }
    ROS_INFO("Map cloud created with %lu points", map_cloud_->size());
}


void DMAPLocalizer::createDMAP() {
    // Clear the DMAP cloud
    dmap_cloud_->clear();

    // Create a KD-tree for obstacle points
    pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (int y = 0; y < map_.info.height; ++y) {
        for (int x = 0; x < map_.info.width; ++x) {
            int index = y * map_.info.width + x;
            if (map_.data[index] > 50) {  // Occupancy threshold for obstacles
                pcl::PointXYZ obstacle_point;
                obstacle_point.x = x * map_.info.resolution + map_.info.origin.position.x;
                obstacle_point.y = y * map_.info.resolution + map_.info.origin.position.y;
                obstacle_point.z = 0.0;  // 2D map, so z = 0
                obstacle_cloud->push_back(obstacle_point);
            }
        }
    }
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(obstacle_cloud);

    // Calculate distance to nearest obstacle for each free space point
    for (int y = 0; y < map_.info.height; ++y) {
        for (int x = 0; x < map_.info.width; ++x) {
            int index = y * map_.info.width + x;
            if (map_.data[index] <= 50) {  // Free space or unknown space
                pcl::PointXYZ free_space_point;
                free_space_point.x = x * map_.info.resolution + map_.info.origin.position.x;
                free_space_point.y = y * map_.info.resolution + map_.info.origin.position.y;
                free_space_point.z = 0.0;

                // Find nearest obstacle using KD-tree
                std::vector<int> indices;
                std::vector<float> distances;
                kdtree.nearestKSearch(free_space_point, 1, indices, distances);

                // Calculate Manhattan distance (adjust for grid-based map)
                float distance = std::abs(free_space_point.x - obstacle_cloud->points[indices[0]].x) +
                                 std::abs(free_space_point.y - obstacle_cloud->points[indices[0]].y);

                // Set the z-coordinate to represent the distance to the nearest obstacle
                free_space_point.z = distance;
                dmap_cloud_->push_back(free_space_point);
            }
        }
    }
    ROS_INFO("DMAP created with %lu points", dmap_cloud_->size());
}


void DMAPLocalizer::publishDMAPCloud() {
    sensor_msgs::PointCloud2 dmap_cloud_msg;
    pcl::toROSMsg(*dmap_cloud_, dmap_cloud_msg);
    dmap_cloud_msg.header.frame_id = map_.header.frame_id;
    dmap_cloud_msg.header.stamp = ros::Time::now();
    dmap_cloud_pub_.publish(dmap_cloud_msg);
    ROS_INFO("Publishing DMAP cloud with %lu points", dmap_cloud_->size());
}

sensor_msgs::LaserScan DMAPLocalizer::createManualLaserScan(const geometry_msgs::Pose& pose) {
    sensor_msgs::LaserScan scan;
    scan.header.frame_id = "base_link";
    scan.header.stamp = ros::Time::now();
    
    scan.angle_min = -M_PI;
    scan.angle_max = M_PI;
    scan.angle_increment = M_PI / 180; // 1-degree resolution
    scan.time_increment = 0.0;
    scan.scan_time = 0.1;
    scan.range_min = 0.1;
    scan.range_max = 30.0;

    size_t num_readings = (scan.angle_max - scan.angle_min) / scan.angle_increment + 1;
    scan.ranges.resize(num_readings);

    for (size_t i = 0; i < num_readings; ++i) {
        float angle = scan.angle_min + i * scan.angle_increment;
        scan.ranges[i] = raycast(pose, angle, scan.range_max);
    }

    return scan;
}

float DMAPLocalizer::raycast(const geometry_msgs::Pose& pose, float angle, float max_range) {
    float global_angle = tf2::getYaw(pose.orientation) + angle;
    float min_range = 0.1;  // Define a minimum range
    
    for (float range = min_range; range <= max_range; range += map_.info.resolution) {
        float x = pose.position.x + range * cos(global_angle);
        float y = pose.position.y + range * sin(global_angle);
        
        int map_x = (x - map_.info.origin.position.x) / map_.info.resolution;
        int map_y = (y - map_.info.origin.position.y) / map_.info.resolution;
        
        if (map_x >= 0 && map_x < map_.info.width && map_y >= 0 && map_y < map_.info.height) {
            int index = map_y * map_.info.width + map_x;
            if (map_.data[index] > 50) {  // Obstacle found
                return range;
            }
        }
    }
    
    return max_range;
}

void DMAPLocalizer::manualScanProcessing() {
    if (initial_pose_.position.x == 0.0 && initial_pose_.position.y == 0.0) {
        ROS_WARN("Initial pose not set. Cannot process scan.");
        return;
    }

    ROS_INFO("Processing manual scan at defined pose: x=%.3f, y=%.3f, yaw=%.3f",
             initial_pose_.position.x, initial_pose_.position.y, tf2::getYaw(initial_pose_.orientation));

    sensor_msgs::LaserScan manual_scan = createManualLaserScan(initial_pose_);
    sensor_msgs::LaserScan::ConstPtr scan_ptr = boost::make_shared<sensor_msgs::LaserScan>(manual_scan);
    convertScanToPointCloud(scan_ptr);

    if (scan_cloud_->empty()) {
        ROS_WARN("Generated scan cloud is empty. Check createManualLaserScan and convertScanToPointCloud functions.");
    } else {
        ROS_INFO("Generated scan cloud with %lu points", scan_cloud_->size());
        
        // Add this debug information
        pcl::PointXYZ min_pt, max_pt;
        pcl::getMinMax3D(*scan_cloud_, min_pt, max_pt);
        ROS_INFO("Scan cloud bounds: X[%.2f, %.2f], Y[%.2f, %.2f], Z[%.2f, %.2f]",
                 min_pt.x, max_pt.x, min_pt.y, max_pt.y, min_pt.z, max_pt.z);
    }

    publishScanCloud();
}



void DMAPLocalizer::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    ROS_INFO("Received laser scan");
    convertScanToPointCloud(msg);
    publishScanCloud();
    performLocalization();
}

void DMAPLocalizer::convertScanToPointCloud(const sensor_msgs::LaserScan::ConstPtr& msg) {
    const sensor_msgs::LaserScan& scan = *msg;
    scan_cloud_->clear();
    
    for (size_t i = 0; i < scan.ranges.size(); ++i) {
        float angle = scan.angle_min + i * scan.angle_increment;
        float max_range = std::min(scan.range_max, 30.0f);  // Limit max range to 30 meters
        
        // Ray-cast from the current pose
        for (float range = scan.range_min; range <= max_range; range += map_.info.resolution) {
            float x = current_pose_.position.x + range * cos(angle);
            float y = current_pose_.position.y + range * sin(angle);
            
            // Check if this point is an obstacle in the map
            int map_x = (x - map_.info.origin.position.x) / map_.info.resolution;
            int map_y = (y - map_.info.origin.position.y) / map_.info.resolution;
            
            if (map_x >= 0 && map_x < map_.info.width && map_y >= 0 && map_y < map_.info.height) {
                int index = map_y * map_.info.width + map_x;
                if (map_.data[index] > 50) {  // Obstacle found
                    pcl::PointXYZ point;
                    point.x = x;
                    point.y = y;
                    point.z = 0.0;
                    scan_cloud_->push_back(point);
                    break;  // Stop ray-casting for this angle
                }
            }
        }
    }
}

void DMAPLocalizer::publishScanCloud() {
    sensor_msgs::PointCloud2 scan_cloud_msg;
    pcl::toROSMsg(*scan_cloud_, scan_cloud_msg);
    scan_cloud_msg.header.frame_id = "map";
    scan_cloud_msg.header.stamp = ros::Time::now();
    scan_cloud_pub_.publish(scan_cloud_msg);
}

void DMAPLocalizer::initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    initial_pose_ = msg->pose.pose;
    current_pose_ = initial_pose_;
    ROS_INFO("Initial pose set: x=%.3f, y=%.3f, yaw=%.3f", 
             current_pose_.position.x, 
             current_pose_.position.y, 
             tf2::getYaw(current_pose_.orientation));

    manualScanProcessing();
    performLocalization();
    
    // Reset the timer to start ICP steps
    icp_timer_.stop();
    icp_timer_.start();
}   

void DMAPLocalizer::visualizeCurrentPose() {
    visualization_msgs::Marker marker;
    marker.header.frame_id = map_.header.frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = "current_pose";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = current_pose_;
    marker.scale.x = 0.5;  // Arrow length
    marker.scale.y = 0.1;  // Arrow width
    marker.scale.z = 0.1;  // Arrow height
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration(0.1);  // Short lifetime to ensure frequent updates

    pose_marker_pub_.publish(marker);
}

void DMAPLocalizer::stepICP(const ros::TimerEvent& event) {
    if (dmap_cloud_->empty() || scan_cloud_->empty()) {
        ROS_WARN_THROTTLE(5, "Empty DMAP or scan cloud, skipping ICP step");
        return;
    }

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(scan_cloud_);
    icp.setInputTarget(dmap_cloud_);
    icp.setMaximumIterations(icp_max_iterations_);
    icp.setTransformationEpsilon(icp_transformation_epsilon_);

    Eigen::Matrix4f initial_guess = Eigen::Matrix4f::Identity();
    initial_guess.block<3,3>(0,0) = Eigen::Quaternionf(current_pose_.orientation.w, 
                                                       current_pose_.orientation.x, 
                                                       current_pose_.orientation.y, 
                                                       current_pose_.orientation.z).toRotationMatrix();
    initial_guess.block<3,1>(0,3) = Eigen::Vector3f(current_pose_.position.x, 
                                                    current_pose_.position.y, 
                                                    current_pose_.position.z);

    pcl::PointCloud<pcl::PointXYZ> aligned_cloud;
    bool converged = false;
    double current_max_distance = icp_max_correspondence_distance_;

    for (int attempt = 0; attempt < 3 && !converged; ++attempt) {
        icp.setMaxCorrespondenceDistance(current_max_distance);
        icp.align(aligned_cloud, initial_guess);

        if (icp.hasConverged()) {
            converged = true;
            Eigen::Matrix4f transformation = icp.getFinalTransformation();
            updatePose(transformation);
            publishPose();
            visualizeCurrentPose();
            publishScanCloud();

            double fitness_score = icp.getFitnessScore();
            ROS_INFO("ICP converged with score: %.4f (max distance: %.2f)", fitness_score, current_max_distance);
            ROS_INFO("Current pose: x=%.3f, y=%.3f, yaw=%.3f", 
                     current_pose_.position.x, 
                     current_pose_.position.y, 
                     tf2::getYaw(current_pose_.orientation));
        } else {
            current_max_distance *= 1.5;  // Increase max distance for next attempt
            ROS_WARN("ICP did not converge, increasing max distance to %.2f", current_max_distance);
        }
    }

    if (!converged) {
        ROS_ERROR("ICP failed to converge after multiple attempts");
    }
}

void DMAPLocalizer::performLocalization() {
    if (dmap_cloud_->empty() || scan_cloud_->empty()) {
        ROS_WARN("Empty DMAP or scan cloud, skipping localization");
        return;
    }

    current_pose_ = initial_pose_;
    
    publishPose();
    //broadcastTransform();

    ROS_INFO("Localization initialized. Use the 'step_icp' topic to perform ICP steps.");
}

void DMAPLocalizer::updatePose(const Eigen::Matrix4f& transformation) {
    Eigen::Vector3f translation = transformation.block<3,1>(0,3);
    Eigen::Matrix3f rotation = transformation.block<3,3>(0,0);
    Eigen::Quaternionf quaternion(rotation);

    current_pose_.position.x = translation.x();
    current_pose_.position.y = translation.y();
    current_pose_.position.z = translation.z();
    current_pose_.orientation.x = quaternion.x();
    current_pose_.orientation.y = quaternion.y();
    current_pose_.orientation.z = quaternion.z();
    current_pose_.orientation.w = quaternion.w();
}

void DMAPLocalizer::publishPose() {
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = map_.header.frame_id;
    pose_msg.pose = current_pose_;
    localized_pose_pub_.publish(pose_msg);
}

// void DMAPLocalizer::broadcastTransform() {
//     geometry_msgs::TransformStamped transform_stamped;
//     transform_stamped.header.stamp = ros::Time::now();
//     transform_stamped.header.frame_id = map_.header.frame_id;
//     transform_stamped.child_frame_id = "map";
//     transform_stamped.transform.translation.x = current_pose_.position.x;
//     transform_stamped.transform.translation.y = current_pose_.position.y;
//     transform_stamped.transform.translation.z = current_pose_.position.z;
//     transform_stamped.transform.rotation = current_pose_.orientation;
//     tf_broadcaster_->sendTransform(transform_stamped);
// }

int main(int argc, char** argv) {
    ros::init(argc, argv, "dmap_localizer");
    DMAPLocalizer localizer;
    
    localizer.run();
    return 0;
}
