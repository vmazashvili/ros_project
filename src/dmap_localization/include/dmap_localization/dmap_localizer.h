#ifndef DMAP_LOCALIZER_H
#define DMAP_LOCALIZER_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h> 
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>  
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <std_msgs/Empty.h>
#include "dmap_localization/dmap_generator.h"
#include <Eigen/SVD>
#include <Eigen/Geometry>

class DMAPLocalizer {
public:
    DMAPLocalizer();
    void run();

private:
    pcl::KdTreeFLANN<pcl::PointXYZ> dmap_kdtree_;
    bool getDMAPValueAndGradient(const Eigen::Vector2f& p_world, float& value, Eigen::Vector2f& gradient);    
    DMAPGenerator dmap_generator_;
    private:
    double kernel_chi2 = 1.0;  // Or any appropriate value
    int inliers_min_ = 10;     // Or any appropriate value
    float damping_ = 0.05f;    // Or any appropriate value


    // ROS
    ros::NodeHandle nh_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    ros::Subscriber map_sub_;
    ros::Subscriber scan_sub_;
    ros::Subscriber initial_pose_sub_;
    ros::Subscriber step_icp_sub_;
    ros::Publisher pose_marker_pub_;
    ros::Publisher localized_pose_pub_;
    ros::Publisher dmap_cloud_pub_;
    ros::Publisher scan_cloud_pub_;

    // Map and localization data
    nav_msgs::OccupancyGrid map_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr dmap_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan_cloud_; // Pointer to the point cloud
    geometry_msgs::Pose current_pose_;
    geometry_msgs::Pose initial_pose_;

    sensor_msgs::LaserScan createManualLaserScan(const geometry_msgs::Pose& pose);
    

    // Parameters
    double dmap_threshold_;
    int icp_max_iterations_;
    double icp_transformation_epsilon_;
    double icp_max_correspondence_distance_;

    float raycast(const geometry_msgs::Pose& pose, float angle, float max_range);
    void initializeROS();
    void visualizeCurrentPose();
    void initializePCL();
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void createMapCloud();
    void createDMAP();
    void publishDMAPCloud();
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void convertScanToPointCloud(const sensor_msgs::LaserScan::ConstPtr& msg);
    void publishScanCloud();
    void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    //void stepICP(const std_msgs::Empty::ConstPtr& msg);
    void performLocalization();
    void updatePose(const Eigen::Isometry2f& transformation);    void publishPose();
    void broadcastTransform();
    void manualScanProcessing();
    void generatePointCloudFromPose(const geometry_msgs::Pose& pose);
    void performICP();
    Eigen::Matrix4f poseToMatrix(const geometry_msgs::Pose& pose);
    Eigen::Isometry2f getInitialGuess();
    bool isConverged(const Eigen::Matrix4f& current, const Eigen::Matrix4f& previous, double epsilon);

};

#endif // DMAP_LOCALIZER_H