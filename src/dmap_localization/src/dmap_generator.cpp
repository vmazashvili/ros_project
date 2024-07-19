#include "dmap_localization/dmap_generator.h"

DMAPGenerator::DMAPGenerator() : dmap_threshold_(0.8) {
    dmap_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
}

void DMAPGenerator::setMap(const nav_msgs::OccupancyGrid& map) {
    map_ = map;
}

void DMAPGenerator::createDMAP() {
    dmap_cloud_->clear();

    pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    createObstacleCloud(obstacle_cloud);

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(obstacle_cloud);

    for (int y = 0; y < map_.info.height; ++y) {
        for (int x = 0; x < map_.info.width; ++x) {
            int index = y * map_.info.width + x;
            if (map_.data[index] <= 50) {  // Free space or unknown space
                pcl::PointXYZ free_space_point;
                free_space_point.x = x * map_.info.resolution + map_.info.origin.position.x;
                free_space_point.y = y * map_.info.resolution + map_.info.origin.position.y;
                free_space_point.z = 0.0;

                std::vector<int> indices;
                std::vector<float> distances;
                kdtree.nearestKSearch(free_space_point, 1, indices, distances);

                float distance = std::abs(free_space_point.x - obstacle_cloud->points[indices[0]].x) +
                                 std::abs(free_space_point.y - obstacle_cloud->points[indices[0]].y);

                free_space_point.z = distance;
                dmap_cloud_->push_back(free_space_point);
            }
        }
    }
    ROS_INFO("DMAP created with %lu points", dmap_cloud_->size());
}

pcl::PointCloud<pcl::PointXYZ>::Ptr DMAPGenerator::getDMAPCloud() const {
    return dmap_cloud_;
}

void DMAPGenerator::createObstacleCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& obstacle_cloud) {
    for (int y = 0; y < map_.info.height; ++y) {
        for (int x = 0; x < map_.info.width; ++x) {
            int index = y * map_.info.width + x;
            if (map_.data[index] > 50) {  // Occupancy threshold for obstacles
                pcl::PointXYZ obstacle_point;
                obstacle_point.x = x * map_.info.resolution + map_.info.origin.position.x;
                obstacle_point.y = y * map_.info.resolution + map_.info.origin.position.y;
                obstacle_point.z = 0.0;  // 2D map, so z = 0
                obstacle_cloud->push_back(obstacle_point);
            }
        }
    }
}

