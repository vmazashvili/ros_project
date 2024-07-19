#ifndef DMAP_GENERATOR_H
#define DMAP_GENERATOR_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>


class DMAPGenerator {
public:
    DMAPGenerator();
    void setMap(const nav_msgs::OccupancyGrid& map);
    void createDMAP();
    pcl::PointCloud<pcl::PointXYZ>::Ptr getDMAPCloud() const;

private:
    nav_msgs::OccupancyGrid map_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr dmap_cloud_;
    double dmap_threshold_;

    void createObstacleCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& obstacle_cloud);
};

#endif // DMAP_GENERATOR_H
