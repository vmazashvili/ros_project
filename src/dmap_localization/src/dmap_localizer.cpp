#include "dmap_localization/dmap_localizer.h"
#include "dmap_localization/dmap_generator.h"
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <vector>
#include <limits>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <visualization_msgs/Marker.h>
#include <tf2/utils.h>
#include "dmap_localization/dmap_localizer.h"


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
    //scan_sub_ = nh_.subscribe("/scan", 1, &DMAPLocalizer::scanCallback, this);
    initial_pose_sub_ = nh_.subscribe("/initialpose", 1, &DMAPLocalizer::initialPoseCallback, this);
    step_icp_sub_ = nh_.subscribe("/step_icp", 1, &DMAPLocalizer::stepICP, this);
    
    localized_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("localized_pose", 1);
    dmap_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("dmap_cloud", 1);
    scan_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("scan_cloud", 1);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>();

    // Load parameters
    nh_.param("dmap_threshold", dmap_threshold_, 0.8);
    nh_.param("icp_max_iterations", icp_max_iterations_, 50);
    nh_.param("icp_transformation_epsilon", icp_transformation_epsilon_, 1e-8);
    nh_.param("icp_max_correspondence_distance", icp_max_correspondence_distance_, 0.1);
}

void DMAPLocalizer::initializePCL() {
    map_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    dmap_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    scan_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
}

void DMAPLocalizer::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    ROS_INFO("Received map");
    map_ = *msg;
    dmap_generator_.setMap(map_);
    dmap_generator_.createDMAP();
    publishDMAPCloud();
}


void DMAPLocalizer::publishDMAPCloud() {
    sensor_msgs::PointCloud2 dmap_cloud_msg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr dmap_cloud = dmap_generator_.getDMAPCloud();
    pcl::toROSMsg(*dmap_cloud, dmap_cloud_msg);
    dmap_cloud_msg.header.frame_id = map_.header.frame_id;
    dmap_cloud_msg.header.stamp = ros::Time::now();
    dmap_cloud_pub_.publish(dmap_cloud_msg);
    ROS_INFO("Publishing DMAP cloud with %lu points", dmap_cloud->size());
}

void DMAPLocalizer::initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    initial_pose_ = msg->pose.pose;
    current_pose_ = initial_pose_;
    ROS_INFO("Initial pose set");

    // Generate a point cloud directly
    generatePointCloudFromPose(current_pose_);
    
    publishScanCloud();
    
    // Call stepICP() with an empty message
    std_msgs::Empty empty_msg;
    stepICP(boost::make_shared<std_msgs::Empty>(empty_msg));
}

// Scan pointcloud with reycasting
void DMAPLocalizer::generatePointCloudFromPose(const geometry_msgs::Pose& pose) {
    scan_cloud_->clear();

    // Set parameters for point cloud generation
    float min_range = 0.1;
    float max_range = 30.0;
    float resolution = map_.info.resolution;
    float pose_yaw = tf2::getYaw(pose.orientation);

    // Generate points in the point cloud
    for (float angle = -M_PI; angle <= M_PI; angle += M_PI / 180) { // 1-degree increments
        float global_angle = pose_yaw + angle;

        for (float range = min_range; range <= max_range; range += resolution) {
            float x = pose.position.x + range * cos(global_angle);
            float y = pose.position.y + range * sin(global_angle);
            
            int map_x = (x - map_.info.origin.position.x) / resolution;
            int map_y = (y - map_.info.origin.position.y) / resolution;
            
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



// void DMAPLocalizer::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
//     ROS_INFO("Received laser scan");
//     convertScanToPointCloud(msg);
//     publishScanCloud();
//     //performLocalization();
// }



void DMAPLocalizer::publishScanCloud() {
    sensor_msgs::PointCloud2 scan_cloud_msg;
    pcl::toROSMsg(*scan_cloud_, scan_cloud_msg);
    scan_cloud_msg.header.frame_id = "map";
    scan_cloud_msg.header.stamp = ros::Time::now();
    scan_cloud_pub_.publish(scan_cloud_msg);
}


// void DMAPLocalizer::visualizeCurrentPose() {
//     visualization_msgs::Marker marker;
//     marker.header.frame_id = map_.header.frame_id;
//     marker.header.stamp = ros::Time::now();
//     marker.ns = "current_pose";
//     marker.id = 0;
//     marker.type = visualization_msgs::Marker::ARROW;
//     marker.action = visualization_msgs::Marker::ADD;
//     marker.pose = current_pose_;
//     marker.scale.x = 0.5;  // Arrow length
//     marker.scale.y = 0.1;  // Arrow width
//     marker.scale.z = 0.1;  // Arrow height
//     marker.color.r = 1.0;
//     marker.color.g = 0.0;
//     marker.color.b = 0.0;
//     marker.color.a = 1.0;
//     marker.lifetime = ros::Duration();

//     pose_marker_pub_.publish(marker);
// }

/*****************************************************************************************************************/

void DMAPLocalizer::stepICP(const std_msgs::Empty::ConstPtr& msg) {
    if (dmap_cloud_->empty() || scan_cloud_->empty()) {
        ROS_WARN("Empty DMAP or scan cloud, skipping ICP step");
        return;
    }

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(scan_cloud_);
    icp.setInputTarget(dmap_cloud_);
    icp.setMaximumIterations(1);  // Only one iteration per step
    icp.setTransformationEpsilon(icp_transformation_epsilon_);
    icp.setMaxCorrespondenceDistance(icp_max_correspondence_distance_);

    Eigen::Matrix4f initial_guess = Eigen::Matrix4f::Identity();
    initial_guess.block<3,3>(0,0) = Eigen::Quaternionf(current_pose_.orientation.w, 
                                                       current_pose_.orientation.x, 
                                                       current_pose_.orientation.y, 
                                                       current_pose_.orientation.z).toRotationMatrix();
    initial_guess.block<3,1>(0,3) = Eigen::Vector3f(current_pose_.position.x, 
                                                    current_pose_.position.y, 
                                                    current_pose_.position.z);

    pcl::PointCloud<pcl::PointXYZ> aligned_cloud;
    icp.align(aligned_cloud, initial_guess);

    if (icp.hasConverged()) {
        Eigen::Matrix4f transformation = icp.getFinalTransformation();
        updatePose(transformation);
        publishPose();
        broadcastTransform();

        double fitness_score = icp.getFitnessScore();
        ROS_INFO("ICP step converged with score: %.4f", fitness_score);
    } else {
        ROS_WARN("ICP step did not converge");
    }
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

void DMAPLocalizer::broadcastTransform() {
    geometry_msgs::TransformStamped transform_stamped;
    transform_stamped.header.stamp = ros::Time::now();
    transform_stamped.header.frame_id = map_.header.frame_id;
    transform_stamped.child_frame_id = "map";
    transform_stamped.transform.translation.x = current_pose_.position.x;
    transform_stamped.transform.translation.y = current_pose_.position.y;
    transform_stamped.transform.translation.z = current_pose_.position.z;
    transform_stamped.transform.rotation = current_pose_.orientation;
    tf_broadcaster_->sendTransform(transform_stamped);
}

/*****************************************************************************************************************/

// void DMAPLocalizer::performLocalization() {
//     if (dmap_cloud_->empty() || scan_cloud_->empty()) {
//         ROS_WARN("Empty DMAP or scan cloud, skipping localization");
//         return;
//     }

//     current_pose_ = initial_pose_;
    
//     publishPose();
//     broadcastTransform();

//     ROS_INFO("Localization initialized. Use the 'step_icp' topic to perform ICP steps.");
// }





int main(int argc, char** argv) {
    ros::init(argc, argv, "dmap_localizer");
    DMAPLocalizer localizer;
    
    localizer.run();
    return 0;
}