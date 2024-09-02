/*
ROS main node named that performs robot localization using
laser scan data in a pre-defined map. 
Subscribes to:
    /map - provides the occupancy grid representing the environment
    /initialpose - provides the initial pose of the robot within the map
    /base_scan - receives laser scan data from the robot's LiDAR sensor
*/

// ROS and standard libraries headers
#include <ros/ros.h>
#include <iostream>
#include <cstdlib>
#include <cmath>
#include <fstream>

// ROS message types
#include <nav_msgs/OccupancyGrid.h> 
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

// Custom headers for the DMap, GridMap, and localization logic
#include <rp_files/dmap.h>
#include <rp_files/grid_map.h>
#include <rp_files/dmap_localizer.h>

//Inizialation of the DMap, localizer and grid
DMap dmap(0,0);
DMapLocalizer localizer;
GridMap grid_map(0.1,1,1);

// Necessary flags
bool map_received = false;
bool init_pose_received = false; 
bool first_scan = true; 

//Vector of obstacles and the canvas for the visualization
std::vector<Vector2f> obstacles;
Canvas canvas; 


// Callback function to handle the reception of the map (OccupancyGrid)
void mapCallBack(const nav_msgs::OccupancyGrid& map){
    if(map_received == false){
        std::cerr << "Map received" << std::endl;

        // Extract map information
        uint32_t width = map.info.width;
        uint32_t height = map.info.height;
        float resolution = map.info.resolution;
        
        // Set up the grid map with the received dimensions and data
        grid_map.resize(height,width);
        grid_map.cells = map.data;
        localizer.setMap(grid_map,1,100);
        localizer.distances.draw(canvas,true);
        localizer.grid_mapping.reset(Vector2f(0,0),resolution);
        
        map_received = true;
    }
    std::cerr << "Dmap ready" << std::endl;
}

// Callback function to handle the reception of the initial pose
void initCallBack(const geometry_msgs::PoseWithCovarianceStamped& init){
    if(map_received == true && init_pose_received == false){
        std::cerr << "Init pose received" << std::endl;
        
        // Extract the initial pose from the received message
        Isometry2f X=Eigen::Isometry2f::Identity();
        Eigen::Quaternionf q(init.pose.pose.orientation.w,init.pose.pose.orientation.x,init.pose.pose.orientation.y,init.pose.pose.orientation.z); 
        
        // Set translation from the initial position
        float x = init.pose.pose.position.x;
        float y = init.pose.pose.position.y;
        X.translation() << x, y;
        
        // Convert quaternion to rotation matrix and set rotation
        X.linear()=q.toRotationMatrix().topLeftCorner<2,2>();
        
        // Set the localizer's pose
        localizer.X=X;
        
        init_pose_received = true;
    }
}

// Callback function to handle the reception of the laser scan data
void laserCallBack(const sensor_msgs::LaserScan& scan){
    if(map_received == true && init_pose_received == true){
        std::vector<Vector2f> scan_endpoints;
        
        // Extract laser scan points and convert to Cartesian coordinates 
        for (size_t i=0; i<scan.ranges.size(); ++i) {
            float alpha=scan.angle_min+i*scan.angle_increment;
            float r=scan.ranges[i];

            // Validate range to ensure it's within the sensor's operational limits
            if (r< scan.range_min || r> scan.range_max)
            continue;
            scan_endpoints.push_back(Vector2f(r*cos(alpha), r*sin(alpha)));
        }

        //Compute localize and print odometry
        localizer.localize(scan_endpoints,10);
        std::cerr << "t: " << localizer.X.translation().transpose() << std::endl;
    }
    return;
}

// Main function to initialize the node and subscribe to topics
int main(int argc, char** argv){
    // Initialize the ROS node with a name "main_node"
    ros::init(argc,argv,"main_node");
    
    //create topic name
    std::string map_topic_name = "/map";
    std::string init_topic_name = "/initialpose";
    std::string laser_topic_name = "/base_scan";

    // Node handles for subscribing to different topics
    ros::NodeHandle n_map; 
    ros::NodeHandle n_init_pose;
    ros::NodeHandle n_laser_scan;

    // Subscribe to topic from topic name
    ros::Subscriber sub = n_map.subscribe<const nav_msgs::OccupancyGrid&>(map_topic_name, 10, mapCallBack);
    ros::Subscriber sub_init_pose = n_init_pose.subscribe<const geometry_msgs::PoseWithCovarianceStamped&>(init_topic_name, 10, initCallBack);
    ros::Subscriber sub_laser_scan = n_laser_scan.subscribe<const sensor_msgs::LaserScan&>(laser_topic_name, 10, laserCallBack);

    // Enter a loop, pumping callbacks, waiting for data, and processing it
    ros::spin();
    return 0;
} 