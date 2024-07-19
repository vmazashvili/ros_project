#include "dmap_localization/dmap_localizer.h"
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>

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
    : nh_("~"), tf_listener_(tf_buffer_), dmap_threshold_(0.8), map_frame_id_("map")
{
	dmap_creation_timer_ = nh_.createTimer(ros::Duration(1.0), &DMapLocalizer::checkAndCreateDMAP, this, false, false);
	nh_.param<std::string>("map_frame_id", map_frame_id_, "map");
	map_sub_ = nh_.subscribe("/map", 1, &DMapLocalizer::mapCallback, this);    
	scan_sub_ = nh_.subscribe("/scan", 1, &DMapLocalizer::scanCallback, this);
    odom_sub_ = nh_.subscribe("odom", 1, &DMapLocalizer::odomCallback, this);
    //initial_pose_sub_ = nh_.subscribe("/initialpose", 1, &DMapLocalizer::initialPoseCallback, this);
    corrected_scan_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>); // Add this line

    localized_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("localized_pose", 1);
	dmap_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("dmap", 1, true);  // The 'true' makes it a latched topic
    scan_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("scan_cloud", 1);
    dmap_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("dmap_cloud", 1, true); 

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
             msg->header.frame_id.c_str(), msg->info.width, msg->info.height); //runs
    map_frame_id_ = msg->header.frame_id;
    map_resolution_ = msg->info.resolution;
    map_origin_ = msg->info.origin.position;

    // Resize and populate the occupancy grid
    occupancy_grid_.resize(msg->info.height, std::vector<int>(msg->info.width));
    for (unsigned int y = 0; y < msg->info.height; ++y) {
        for (unsigned int x = 0; x < msg->info.width; ++x) {
            occupancy_grid_[y][x] = msg->data[y * msg->info.width + x];
        }
    }

    // Instead of directly calling createDMAP(), we use the timer to periodically call checkAndCreateDMAP()
    if (!dmap_creation_timer_.hasStarted()) {
        dmap_creation_timer_.start();
    }
}


void DMapLocalizer::checkAndCreateDMAP(const ros::TimerEvent&)
{
	if (occupancy_grid_.empty()) {
		ROS_WARN_THROTTLE(10, "Occupancy grid is empty. Waiting for map data...");
		return;
	}

	if (dmap_.empty()) {
		ROS_INFO("Creating DMAP..."); //runs
		createDMAP();
	} else {
		// DMAP already created, stop the timer
		dmap_creation_timer_.stop();
		ROS_INFO("DMAP created!"); //runs
	}

    sensor_msgs::PointCloud2 dmap_cloud_msg;
    pcl::toROSMsg(*dmap_cloud_, dmap_cloud_msg);
    dmap_cloud_msg.header.frame_id = map_frame_id_; // Set the correct frame_id
    dmap_cloud_msg.header.stamp = ros::Time::now();
    
    // Publish the PointCloud2 message
    dmap_cloud_pub_.publish(dmap_cloud_msg);
}


void DMapLocalizer::createDMAP()
{
    dmap_.resize(occupancy_grid_.size(), std::vector<float>(occupancy_grid_[0].size(), std::numeric_limits<float>::max()));

    // First pass: top-left to bottom-right
	ROS_INFO("Creating DMAP: First pass: top-left to bottom-right."); //runs
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
	ROS_INFO("Creating DMAP: Second pass: top-left to bottom-right."); //runs
    for (int y = occupancy_grid_.size() - 1; y >= 0; --y) {
        for (int x = occupancy_grid_[y].size() - 1; x >= 0; --x) {
            if (y < occupancy_grid_.size() - 1) dmap_[y][x] = std::min(dmap_[y][x], dmap_[y+1][x] + 1);
            if (x < occupancy_grid_[y].size() - 1) dmap_[y][x] = std::min(dmap_[y][x], dmap_[y][x+1] + 1);
        }
    }

    // Scale distances by map resolution
	ROS_INFO("Creating DMAP: Scaling distances by map resolution."); //runs
    for (auto& row : dmap_) {
        for (auto& cell : row) {
            cell *= map_resolution_;
        }
    }

    // Publish DMAP for visualization
	ROS_INFO("Publishing DMAP"); //runs
    nav_msgs::OccupancyGrid dmap_msg;
    dmap_msg.header.stamp = ros::Time::now();
	dmap_msg.header.frame_id = map_frame_id_;
    //dmap_msg.header.frame_id = "map";
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
    ROS_INFO("Translating DMAP to point cloud with gradient density...");
    dmap_cloud_->clear();

    int min_points = 1; // Minimum number of points in free space
    int max_points = 10; // Maximum number of points near obstacles

    float max_distance = 0.0;
    for (const auto& row : dmap_) {
        for (const auto& cell : row) {
            if (cell != std::numeric_limits<float>::max()) {
                max_distance = std::max(max_distance, cell);
            }
        }
    }

    for (size_t y = 0; y < dmap_.size(); ++y) {
        for (size_t x = 0; x < dmap_[y].size(); ++x) {
            float distance = dmap_[y][x];
            if (distance != std::numeric_limits<float>::max()) {
                // Determine number of points based on distance (closer = more points)
                int num_points = static_cast<int>(max_points * (1.0 - (distance / max_distance)));
                num_points = std::max(min_points, num_points); // Ensure at least min_points
                
                for (int i = 0; i < num_points; ++i) {
                    pcl::PointXYZ point;
                    point.x = x * map_resolution_ + map_origin_.x;
                    point.y = y * map_resolution_ + map_origin_.y;
                    point.z = 0;

                    // Optional: Add a small random offset to create a gradient effect
                    float offset_x = (static_cast<float>(rand()) / RAND_MAX - 0.5f) * map_resolution_;
                    float offset_y = (static_cast<float>(rand()) / RAND_MAX - 0.5f) * map_resolution_;

                    point.x += offset_x;
                    point.y += offset_y;

                    dmap_cloud_->push_back(point);
                }
            }
        }
    }

    dmap_cloud_->width = dmap_cloud_->points.size();
    dmap_cloud_->height = 1;
    dmap_cloud_->is_dense = true;
    ROS_INFO("Generated DMAP cloud with %ld points", dmap_cloud_->points.size());
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
    ROS_INFO("Received laser scan with %zu ranges", msg->ranges.size());

    // Convert laser scan to point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    for (size_t i = 0; i < msg->ranges.size(); ++i) {
        if (std::isfinite(msg->ranges[i])) {
            float angle = msg->angle_min + i * msg->angle_increment;
            pcl::PointXYZ point;
            point.x = msg->ranges[i] * cos(angle);
            point.y = msg->ranges[i] * sin(angle);
            point.z = 0.0; // Assuming 2D laser scan
            scan_cloud->push_back(point);
        }
    }

    // Align scan with map using ICP
    //Eigen::Matrix4f transform = alignScan(*scan_cloud);

    // Process the point cloud here, e.g., filtering, downsampling, etc.
    // ...

    // Publish the point cloud (optional)
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*scan_cloud, output);
    output.header.frame_id = msg->header.frame_id;
    scan_cloud_pub_.publish(output);
    //publishPose(transform);
}


// Eigen::Matrix4f DMapLocalizer::alignScan(const pcl::PointCloud<pcl::PointXYZ>& scan_cloud) {
//     pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
//     icp.setInputSource(scan_cloud.makeShared());
//     icp.setInputTarget(map_cloud_);

//     pcl::PointCloud<pcl::PointXYZ> aligned_cloud;
//     icp.align(aligned_cloud);

//     if (icp.hasConverged()) {
//         ROS_INFO_STREAM("ICP converged with fitness score: " << icp.getFitnessScore());
//         return icp.getFinalTransformation();
//     } else {
//         ROS_WARN("ICP did not converge.");
    
//     // Handle ICP failure (e.g., return identity matrix or previous transform)
//     return Eigen::Matrix4f::Identity();
//     }
// }


void DMapLocalizer::compensateLaserScan(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    // Assuming linear velocity is in m/s and angular velocity in rad/s
    double scan_time = msg->scan_time; // Total time for one scan
    double angle_increment = msg->angle_increment;

    for (size_t i = 0; i < msg->ranges.size(); ++i) {
        double time_offset = (i * angle_increment) / msg->angle_increment;
        double delta_x = velocity.linear.x * time_offset;
        double delta_y = velocity.linear.y * time_offset;
        double delta_theta = velocity.angular.z * time_offset;

        // Correcting the position
        float angle = msg->angle_min + i * angle_increment + delta_theta;
        pcl::PointXYZ point;
        point.x = (msg->ranges[i] * cos(angle)) + delta_x;
        point.y = (msg->ranges[i] * sin(angle)) + delta_y;
        point.z = 0;
        corrected_scan_cloud_->push_back(point);
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
    velocity = msg->twist.twist; // Update the velocity
}


// // I dont think I need this function anymore. performLocalization() sets initial pose by itself
// void DMapLocalizer::initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
// {
//     initial_pose_ = msg->pose.pose;
//     //initial_pose_set = true;
//     ROS_INFO("Initial pose set."); //runs only when I set an initial pose through RViz
// }

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

    // icp.setMaxCorrespondenceDistance(1);  // Adjust as needed
    // icp.setTransformationEpsilon(1e-9);     // Adjust as needed
    // icp.setMaximumIterations(100);           // Adjust as needed


    Eigen::Matrix4f initial_guess = Eigen::Matrix4f::Identity();
    initial_guess(0, 3) = current_odom_.pose.pose.position.x;
    initial_guess(1, 3) = current_odom_.pose.pose.position.y;

    pcl::PointCloud<pcl::PointXYZ> aligned_cloud;
    icp.align(aligned_cloud, initial_guess);

    if (icp.hasConverged()) {
        double fitness_score = icp.getFitnessScore();
        ROS_INFO("ICP converged with score: %.4f", fitness_score);

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

        if (poseChanged(localized_pose.pose)) {
            localized_pose_pub_.publish(localized_pose);
            ROS_INFO("Published localized pose: x=%.2f, y=%.2f", localized_pose.pose.position.x, localized_pose.pose.position.y); //runs
            broadcastTransform(localized_pose);
            last_published_pose_ = localized_pose.pose;
        }
    } else {
        ROS_WARN("ICP did not converge");
    }
}

const double MIN_POSE_CHANGE = 0.5;
const double MIN_ORIENTATION_CHANGE = 0.5; // For checking orientation changes

bool DMapLocalizer::poseChanged(const geometry_msgs::Pose& new_pose)
{
    double dx = new_pose.position.x - last_published_pose_.position.x;
    double dy = new_pose.position.y - last_published_pose_.position.y;
    double d_position = sqrt(dx*dx + dy*dy);
    
    double dq = std::fabs(new_pose.orientation.z - last_published_pose_.orientation.z); // Adjust if using full quaternion

    return (d_position > MIN_POSE_CHANGE || dq > MIN_ORIENTATION_CHANGE);
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

    //ROS_INFO("broadcastTransform works!"); //WORKS! 
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



