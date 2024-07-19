#include "dmap_localization/laser_scan_processor.h"

LaserScanProcessor::LaserScanProcessor() {
    // Initialization, if needed
}

void LaserScanProcessor::convertScanToPointCloud(const sensor_msgs::LaserScan::ConstPtr& scan_msg,
                                                 pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    cloud->clear();
    double angle = scan_msg->angle_min;

    for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
        float range = scan_msg->ranges[i];
        if (range >= scan_msg->range_min && range <= scan_msg->range_max) {
            pcl::PointXYZ point;
            point.x = range * cos(angle);
            point.y = range * sin(angle);
            point.z = 0.0;
            cloud->points.push_back(point);
        }
        angle += scan_msg->angle_increment;
    }
}

float LaserScanProcessor::raycast(const geometry_msgs::Pose& pose, float angle, float max_range) {
    // Implement raycasting logic, if needed, otherwise leave it unimplemented
    // For now, just return the max_range as a placeholder
    return max_range;
}

