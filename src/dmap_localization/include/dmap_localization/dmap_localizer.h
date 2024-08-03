#ifndef DMAP_LOCALIZER_H
#define DMAP_LOCALIZER_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

class DMapLocalizer
{
public:
    DMapLocalizer();

private:
	const double MIN_POSE_CHANGE = 0.01;  // 1 cm
    const double MIN_SCAN_INTERVAL = 0.1;  // 100 ms
	ros::Time last_scan_time_;
	bool poseChanged(const geometry_msgs::Pose& new_pose);
	void checkAndCreateDMAP(const ros::TimerEvent&);
    ros::Timer dmap_creation_timer_;
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    
    void createDMAP();
    void createDMAPPointCloud();
    void performLocalization();
    void broadcastTransform(const geometry_msgs::PoseStamped& pose);

    ros::NodeHandle nh_;
    ros::Subscriber map_sub_;
    ros::Subscriber scan_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber initial_pose_sub_;
    ros::Publisher localized_pose_pub_;
    ros::Publisher dmap_pub_;

    std::vector<std::vector<int>> occupancy_grid_;
    std::vector<std::vector<float>> dmap_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr dmap_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan_cloud_;

    nav_msgs::Odometry current_odom_;
    geometry_msgs::Pose initial_pose_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    float map_resolution_;
	geometry_msgs::Pose last_published_pose_;
    geometry_msgs::Point map_origin_;
    float dmap_threshold_;
    std::string map_frame_id_; 
};

#endif // DMAP_LOCALIZER_H