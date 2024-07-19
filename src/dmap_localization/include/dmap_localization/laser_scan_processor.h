#ifndef LASER_SCAN_PROCESSOR_H
#define LASER_SCAN_PROCESSOR_H

#include <sensor_msgs/LaserScan.h>
#include <pcl/point_cloud.h>

class LaserScanProcessor {
public:
    LaserScanProcessor();
    void convertScanToPointCloud(const sensor_msgs::LaserScan::ConstPtr& scan_msg,
                                 pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

private:
    float raycast(const geometry_msgs::Pose& pose, float angle, float max_range);
};

#endif  // LASER_SCAN_PROCESSOR_H

