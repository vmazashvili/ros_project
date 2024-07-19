//#include "dmap_localization/custom_icp.h"
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
#include <Eigen/Geometry>



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
    //step_icp_sub_ = nh_.subscribe("/step_icp", 1, &DMAPLocalizer::stepICP, this);
    
    localized_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("localized_pose", 1);
    dmap_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("dmap_cloud", 1);
    scan_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("scan_cloud", 1);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>();

    // Load parameters
    nh_.param("dmap_threshold", dmap_threshold_, 0.8);
    nh_.param("icp_max_iterations", icp_max_iterations_, 100);
    nh_.param("icp_transformation_epsilon", icp_transformation_epsilon_, 1e-5);
    nh_.param("icp_max_correspondence_distance", icp_max_correspondence_distance_, 1.0);
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
    dmap_cloud_ = dmap_generator_.getDMAPCloud();
    
    dmap_kdtree_.setInputCloud(dmap_cloud_);
    
    publishDMAPCloud();
}

void DMAPLocalizer::publishDMAPCloud() {
    sensor_msgs::PointCloud2 dmap_cloud_msg;
    pcl::toROSMsg(*dmap_cloud_, dmap_cloud_msg);  // Use dmap_cloud_ directly
    dmap_cloud_msg.header.frame_id = map_.header.frame_id;
    dmap_cloud_msg.header.stamp = ros::Time::now();
    dmap_cloud_pub_.publish(dmap_cloud_msg);
    ROS_INFO("Publishing DMAP cloud with %lu points", dmap_cloud_->size());
}

void DMAPLocalizer::initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    if (dmap_cloud_->empty()) {
        ROS_WARN("DMAP not ready yet. Skipping initialization.");
        return;
    }

    initial_pose_ = msg->pose.pose;
    current_pose_ = initial_pose_;
    ROS_INFO("Initial pose set");

    generatePointCloudFromPose(current_pose_);
    publishScanCloud();
    
    performICP();
}

// Scan pointcloud with reycasting
void DMAPLocalizer::generatePointCloudFromPose(const geometry_msgs::Pose& pose) {
    scan_cloud_->clear();

    // Set parameters for point cloud generation
    float min_range = 0.1;
    float max_range = 50.0;
    float resolution = map_.info.resolution;
    float pose_yaw = tf2::getYaw(pose.orientation);

    // Generate points in the point cloud
    for (float angle = -M_PI; angle <= M_PI; angle += M_PI / 10000) { // 1-degree increments
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
    ROS_INFO("Generated scan cloud with %lu points", scan_cloud_->size());
}

void DMAPLocalizer::publishScanCloud() {
    sensor_msgs::PointCloud2 scan_cloud_msg;
    pcl::toROSMsg(*scan_cloud_, scan_cloud_msg);
    scan_cloud_msg.header.frame_id = "map";
    scan_cloud_msg.header.stamp = ros::Time::now();
    scan_cloud_pub_.publish(scan_cloud_msg);
}

/*****************************************************************************************************************/

void DMAPLocalizer::performICP() {
    if (dmap_cloud_->empty() || scan_cloud_->empty()) {
        ROS_WARN("Empty DMAP or scan cloud, skipping ICP");
        return;
    }

    Eigen::Isometry2f X = getInitialGuess();

    for (int iteration = 0; iteration < icp_max_iterations_; ++iteration) {
        Eigen::Matrix3f H = Eigen::Matrix3f::Zero();
        Eigen::Vector3f b = Eigen::Vector3f::Zero();
        int inliers = 0;
        float chi2 = 0;

        for (const auto& point : scan_cloud_->points) {
            Eigen::Vector2f m(point.x, point.y);
            Eigen::Vector2f p_world = X * m;
            
            float dmap_value;
            Eigen::Vector2f dmap_gradient;
            if (!getDMAPValueAndGradient(p_world, dmap_value, dmap_gradient)) {
                continue;
            }

            float e = dmap_value;
            float e2 = e * e;
            float lambda = 1.0f;
            if (e2 > kernel_chi2) {
                lambda = std::sqrt(kernel_chi2 / e2);
            }

            Eigen::Matrix<float, 2, 3> J_icp;
            J_icp.block<2, 2>(0, 0).setIdentity();
            J_icp.col(2) << -p_world.y(), p_world.x();

            Eigen::Matrix<float, 1, 2> J_dmap = dmap_gradient.transpose();

            Eigen::Matrix<float, 1, 3> J = J_dmap * J_icp;

            H += lambda * J.transpose() * J;
            b += lambda * J.transpose() * e;
            chi2 += e2;
            ++inliers;
        }

        H += Eigen::Matrix3f::Identity() * damping_;
        Eigen::Vector3f dx = H.ldlt().solve(-b);

        Eigen::Isometry2f dX = Eigen::Isometry2f::Identity();
        dX.translation() << dx.x(), dx.y();
        dX.linear() = Eigen::Rotation2Df(dx.z()).matrix();
        X = dX * X;

        ROS_INFO("Iteration: %d, chi2: %f, inliers: %d/%lu", 
                 iteration, chi2, inliers, scan_cloud_->size());

        if (inliers < inliers_min_) {
            ROS_WARN("Not enough inliers, stopping ICP");
            break;
        }

        // Check for convergence
        if (dx.norm() < icp_transformation_epsilon_) {
            ROS_INFO("ICP converged after %d iterations", iteration + 1);
            break;
        }
    }

    updatePose(X);
    publishPose();
}

bool DMAPLocalizer::getDMAPValueAndGradient(const Eigen::Vector2f& p_world, float& value, Eigen::Vector2f& gradient) {
    pcl::PointXYZ search_point(p_world.x(), p_world.y(), 0);
    
    std::vector<int> nearest_indices(1);
    std::vector<float> nearest_distances(1);
    
    if (dmap_kdtree_.nearestKSearch(search_point, 1, nearest_indices, nearest_distances) > 0) {
        const pcl::PointXYZ& nearest_point = dmap_cloud_->points[nearest_indices[0]];
        value = nearest_point.z;
        
        // Compute gradient using central differences
        float dx = map_.info.resolution;  // Use map resolution instead of fixed value
        float dy = map_.info.resolution;
        
        pcl::PointXYZ px_plus(p_world.x() + dx, p_world.y(), 0);
        pcl::PointXYZ px_minus(p_world.x() - dx, p_world.y(), 0);
        pcl::PointXYZ py_plus(p_world.x(), p_world.y() + dy, 0);
        pcl::PointXYZ py_minus(p_world.x(), p_world.y() - dy, 0);
        
        std::vector<int> ix_plus(1), ix_minus(1), iy_plus(1), iy_minus(1);
        std::vector<float> dx_plus_dist(1), dx_minus_dist(1), dy_plus_dist(1), dy_minus_dist(1);
        
        dmap_kdtree_.nearestKSearch(px_plus, 1, ix_plus, dx_plus_dist);
        dmap_kdtree_.nearestKSearch(px_minus, 1, ix_minus, dx_minus_dist);
        dmap_kdtree_.nearestKSearch(py_plus, 1, iy_plus, dy_plus_dist);
        dmap_kdtree_.nearestKSearch(py_minus, 1, iy_minus, dy_minus_dist);
        
        float vx_plus = dmap_cloud_->points[ix_plus[0]].z;
        float vx_minus = dmap_cloud_->points[ix_minus[0]].z;
        float vy_plus = dmap_cloud_->points[iy_plus[0]].z;
        float vy_minus = dmap_cloud_->points[iy_minus[0]].z;
        
        gradient.x() = (vx_plus - vx_minus) / (2 * dx);
        gradient.y() = (vy_plus - vy_minus) / (2 * dy);
        
        return true;
    }
    
    return false;
}

Eigen::Isometry2f DMAPLocalizer::getInitialGuess() {
    // Use the center of the map as the initial position
    float center_x = map_.info.origin.position.x + (map_.info.width * map_.info.resolution) / 2.0;
    float center_y = map_.info.origin.position.y + (map_.info.height * map_.info.resolution) / 2.0;

    // Use the user-defined orientation or a default orientation
    float initial_yaw = tf2::getYaw(initial_pose_.orientation);

    Eigen::Isometry2f initial_guess = Eigen::Isometry2f::Identity();
    initial_guess.translation() << center_x, center_y;
    initial_guess.linear() = Eigen::Rotation2Df(initial_yaw).matrix();

    return initial_guess;
}

bool DMAPLocalizer::isConverged(const Eigen::Matrix4f& current, const Eigen::Matrix4f& previous, double epsilon) {
    double rot_epsilon = 2 * epsilon;
    double trans_epsilon = epsilon;

    Eigen::Matrix3f R = current.block<3, 3>(0, 0);
    Eigen::Matrix3f R_prev = previous.block<3, 3>(0, 0);
    Eigen::Vector3f t = current.block<3, 1>(0, 3);
    Eigen::Vector3f t_prev = previous.block<3, 1>(0, 3);

    double rot_error = (R * R_prev.transpose()).diagonal().array().abs().sum();
    double trans_error = (t - t_prev).norm();

    return (rot_error < rot_epsilon && trans_error < trans_epsilon);
}

Eigen::Matrix4f DMAPLocalizer::poseToMatrix(const geometry_msgs::Pose& pose) {
    Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
    matrix.block<3,3>(0,0) = Eigen::Quaternionf(pose.orientation.w, 
                                                pose.orientation.x, 
                                                pose.orientation.y, 
                                                pose.orientation.z).toRotationMatrix();
    matrix.block<3,1>(0,3) = Eigen::Vector3f(pose.position.x, 
                                             pose.position.y, 
                                             pose.position.z);
    return matrix;
}

void DMAPLocalizer::updatePose(const Eigen::Isometry2f& transformation) {
    Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
    matrix.block<2,2>(0,0) = transformation.rotation();
    matrix.block<2,1>(0,3) = transformation.translation();

    current_pose_.position.x = matrix(0,3);
    current_pose_.position.y = matrix(1,3);
    current_pose_.position.z = 0.0;

    Eigen::Quaternionf q(matrix.block<3,3>(0,0));
    current_pose_.orientation.x = q.x();
    current_pose_.orientation.y = q.y();
    current_pose_.orientation.z = q.z();
    current_pose_.orientation.w = q.w();
}

void DMAPLocalizer::publishPose() {
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = map_.header.frame_id;
    pose_msg.pose = current_pose_;
    localized_pose_pub_.publish(pose_msg);
}


/*****************************************************************************************************************/

int main(int argc, char** argv) {
    ros::init(argc, argv, "dmap_localizer");
    DMAPLocalizer localizer;
    
    localizer.run();
    return 0;
}