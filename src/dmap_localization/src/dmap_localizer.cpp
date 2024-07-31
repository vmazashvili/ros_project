#include "dmap_localization/dmap_localizer.h"
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
//#include <opencv2/opencv.hpp>

/*
- DMapLocalizer Constructor initializes the node handle `nh_`
- Sets up subscribers for the laser scan, odometry, map topics
- Initializes the publishers and point clouds. 
  - The node listens to:
    - `map`
    - `scan`
    - `odom`
  - publishes:
    - `localized_pose` (estimated pose)
*/ 
DMapLocalizer::DMapLocalizer()
    : nh_("~"), tf_listener_(tf_buffer_), dmap_threshold_(0.5), map_frame_id_("map")
{
	nh_.param<std::string>("map_frame_id", map_frame_id_, "map");
	map_sub_ = nh_.subscribe("/map", 1, &DMapLocalizer::mapCallback, this);    scan_sub_ = nh_.subscribe("scan", 1, &DMapLocalizer::scanCallback, this);
    odom_sub_ = nh_.subscribe("odom", 1, &DMapLocalizer::odomCallback, this);
    initial_pose_sub_ = nh_.subscribe("/initialpose", 1, &DMapLocalizer::initialPoseCallback, this);

    localized_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("localized_pose", 1);
    dmap_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("dmap", 1);

    dmap_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    scan_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
}

/*
- Converts the received occupancy grid map into a point cloud representation `map_cloud_`
- Iterates over the grid cells, checks if the cell is occupied
- Adds the corresponding world coordinates to the `map_cloud_`

- This point cloud is later used in ICP algorithm to alighn the laser scan data with the map
*/
void DMapLocalizer::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    ROS_INFO("Received map with frame_id: %s, width: %d, height: %d", 
             msg->header.frame_id.c_str(), msg->info.width, msg->info.height);
    map_frame_id_ = msg->header.frame_id;
    map_resolution_ = msg->info.resolution;
    map_origin_ = msg->info.origin.position;

    occupancy_grid_.resize(msg->info.height, std::vector<int>(msg->info.width));
    for (unsigned int y = 0; y < msg->info.height; ++y) {
        for (unsigned int x = 0; x < msg->info.width; ++x) {
            occupancy_grid_[y][x] = msg->data[y * msg->info.width + x];
        }
    }
    createDMAP();
    ROS_INFO("DMAP created and published");
}

void DMapLocalizer::createDMAP()
{
    dmap_.resize(occupancy_grid_.size(), std::vector<float>(occupancy_grid_[0].size(), std::numeric_limits<float>::max()));

    // First pass: top-left to bottom-right
    for (size_t y = 0; y < occupancy_grid_.size(); ++y) {
        for (size_t x = 0; x < occupancy_grid_[y].size(); ++x) {
            if (occupancy_grid_[y][x] > 50) {
                dmap_[y][x] = 0;
            } else {
                if (y > 0) dmap_[y][x] = std::min(dmap_[y][x], dmap_[y-1][x] + 1);
                if (x > 0) dmap_[y][x] = std::min(dmap_[y][x], dmap_[y][x-1] + 1);
            }
        }
    }

    // Second pass: bottom-right to top-left
    for (int y = occupancy_grid_.size() - 1; y >= 0; --y) {
        for (int x = occupancy_grid_[y].size() - 1; x >= 0; --x) {
            if (y < occupancy_grid_.size() - 1) dmap_[y][x] = std::min(dmap_[y][x], dmap_[y+1][x] + 1);
            if (x < occupancy_grid_[y].size() - 1) dmap_[y][x] = std::min(dmap_[y][x], dmap_[y][x+1] + 1);
        }
    }

    // Scale distances by map resolution
    for (auto& row : dmap_) {
        for (auto& cell : row) {
            cell *= map_resolution_;
        }
    }

    // Publish DMAP for visualization
    nav_msgs::OccupancyGrid dmap_msg;
    dmap_msg.header.stamp = ros::Time::now();
	dmap_msg.header.frame_id = map_frame_id_;
    dmap_msg.header.frame_id = "map";
    dmap_msg.info.resolution = map_resolution_;
    dmap_msg.info.width = occupancy_grid_[0].size();
    dmap_msg.info.height = occupancy_grid_.size();
    dmap_msg.info.origin = geometry_msgs::Pose();
    dmap_msg.info.origin.position = map_origin_;

    dmap_msg.data.resize(dmap_.size() * dmap_[0].size());
    float max_dist = 0;
    for (const auto& row : dmap_) {
        for (const auto& cell : row) {
            max_dist = std::max(max_dist, cell);
        }
    }

    size_t i = 0;
    for (const auto& row : dmap_) {
        for (const auto& cell : row) {
            dmap_msg.data[i++] = static_cast<int8_t>((1.0 - std::min(cell / max_dist, static_cast<float>(1.0))) * 100);
        }
    }

    dmap_pub_.publish(dmap_msg);
    createDMAPPointCloud();
}

void DMapLocalizer::createDMAPPointCloud()
{
    dmap_cloud_->clear();
    for (size_t y = 0; y < dmap_.size(); ++y) {
        for (size_t x = 0; x < dmap_[y].size(); ++x) {
            if (dmap_[y][x] < dmap_threshold_) {
                pcl::PointXYZ point;
                point.x = x * map_resolution_ + map_origin_.x;
                point.y = y * map_resolution_ + map_origin_.y;
                point.z = 0;
                dmap_cloud_->push_back(point);
            }
        }
    }
    dmap_cloud_->width = dmap_cloud_->points.size();
    dmap_cloud_->height = 1;
    dmap_cloud_->is_dense = true;
}

/*
- Processes the incoming laser scan data
- Converts is into a point cloud
- Checks if the range data is finite
- Calculates the corresponding (x, y) coordinates in the robot's frame
- Adds them to `scan_cloud`
- Once the scan data is available and `map_cloud_` is not empty, 
  it estimates robot's pose by calling `performLocalization()`
*/
void DMapLocalizer::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    ROS_INFO("Received a laser scan with %ld ranges in frame %s", msg->ranges.size(), msg->header.frame_id.c_str());
    scan_cloud_->clear();
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
        if (std::isfinite(msg->ranges[i])) {
            float angle = msg->angle_min + i * msg->angle_increment;
            pcl::PointXYZ point;
            point.x = msg->ranges[i] * cos(angle);
            point.y = msg->ranges[i] * sin(angle);
            point.z = 0;
            scan_cloud_->push_back(point);
        }
    }
    scan_cloud_->width = scan_cloud_->points.size();
    scan_cloud_->height = 1;
    scan_cloud_->is_dense = true;

    ROS_INFO("Created scan cloud with %ld points", scan_cloud_->points.size());

    if (!dmap_cloud_->empty()) {
        performLocalization();
    } else {
        ROS_WARN("DMAP cloud is empty, skipping localization");
    }
}

/*
- Updates the current odometry data `current_odom_` with the latest poition and orientation
- This data can be used for dead reckoning or as an initial guess for localization
*/
void DMapLocalizer::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    ROS_INFO("Received odometry data");
    current_odom_ = *msg;
}

void DMapLocalizer::initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    initial_pose_ = msg->pose.pose;
    //initial_pose_set = true;
    ROS_INFO("Initial pose set.");
}

/*
- Alligns the `scan_cloud_` laser scan data with `map_cloud_` map data
- Using the Iterative Closest Point (ICP) algorithm
- If the alignment converges, the function:
  - Extracts the transformation matrix
  - Converts it to a `geometry_msgs::PoseStamped` message
  - Publishes the pose
    - Also calls `broadcastTransform` to:
      - Broadcast the transform between the "map" and "base_link" frames
*/
void DMapLocalizer::performLocalization()
{
    if (scan_cloud_->empty()) {
        ROS_WARN("Empty scan cloud, skipping localization");
        return;
    }
    if (dmap_cloud_->empty()) {
        ROS_WARN("Empty DMAP cloud, skipping localization");
        return;
    }

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(scan_cloud_);
    icp.setInputTarget(dmap_cloud_);

    Eigen::Matrix4f initial_guess = Eigen::Matrix4f::Identity();
    initial_guess(0, 3) = current_odom_.pose.pose.position.x;
    initial_guess(1, 3) = current_odom_.pose.pose.position.y;

    pcl::PointCloud<pcl::PointXYZ> aligned_cloud;
    icp.align(aligned_cloud, initial_guess);

    if (icp.hasConverged()) {
        Eigen::Matrix4f transformation = icp.getFinalTransformation();

        geometry_msgs::PoseStamped localized_pose;
        localized_pose.header.stamp = ros::Time::now();
        localized_pose.header.frame_id = map_frame_id_;

        localized_pose.pose.position.x = transformation(0, 3);
        localized_pose.pose.position.y = transformation(1, 3);
        localized_pose.pose.position.z = 0;

        Eigen::Quaternionf q(Eigen::Matrix3f(transformation.block<3, 3>(0, 0)));
        localized_pose.pose.orientation.x = q.x();
        localized_pose.pose.orientation.y = q.y();
        localized_pose.pose.orientation.z = q.z();
        localized_pose.pose.orientation.w = q.w();

        localized_pose_pub_.publish(localized_pose);
        ROS_INFO("Published localized pose: x=%.2f, y=%.2f", localized_pose.pose.position.x, localized_pose.pose.position.y);
        broadcastTransform(localized_pose);
    } else {
        ROS_WARN("ICP did not converge");
    }
}

/*
- Broadcasts the transfrom from the "map" frame to the "base_link" frame,
  based on the calculated localized pose
- This broadcast allows other nodes to use the transform data for various tasks, 
  such as visualizing the robot's position in RViz
*/
void DMapLocalizer::broadcastTransform(const geometry_msgs::PoseStamped& pose)
{
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = pose.header.stamp;
    transformStamped.header.frame_id = map_frame_id_;
    transformStamped.child_frame_id = "base_link";

    transformStamped.transform.translation.x = pose.pose.position.x;
    transformStamped.transform.translation.y = pose.pose.position.y;
    transformStamped.transform.translation.z = pose.pose.position.z;
    transformStamped.transform.rotation = pose.pose.orientation;

    tf_broadcaster_.sendTransform(transformStamped);
}

/*
- Initializes the ROS node and the instance of `DMapLocalizer`
- Calls `ros::spin()` to keep the node running, lietening for incoming
  messages and calling the appropriate callbacks
*/
int main(int argc, char** argv)
{
    ros::init(argc, argv, "dmap_localizer");
    DMapLocalizer localizer;
    ros::spin();
    return 0;
}