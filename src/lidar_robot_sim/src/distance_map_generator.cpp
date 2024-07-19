#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <vector>
#include <cmath>

class DistanceMapGenerator
{
public:
    DistanceMapGenerator() : nh_("~")
    {
        map_sub_ = nh_.subscribe("/map", 1, &DistanceMapGenerator::mapCallback, this);
        distance_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/distance_map", 1, true);
    }

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map)
    {
        ROS_INFO("Received map, generating distance map...");
        nav_msgs::OccupancyGrid distance_map = *map;
        std::vector<std::pair<int, int>> obstacles;

        // Extract obstacles
        for (int y = 0; y < map->info.height; ++y)
        {
            for (int x = 0; x < map->info.width; ++x)
            {
                int index = y * map->info.width + x;
                if (map->data[index] > 50)  // Assuming > 50 is occupied
                {
                    obstacles.push_back(std::make_pair(x, y));
                }
            }
        }

        // Calculate distances
        for (int y = 0; y < map->info.height; ++y)
        {
            for (int x = 0; x < map->info.width; ++x)
            {
                float min_dist = std::numeric_limits<float>::max();
                for (const auto& obstacle : obstacles)
                {
                    float dx = x - obstacle.first;
                    float dy = y - obstacle.second;
                    float dist = std::sqrt(dx*dx + dy*dy);
                    min_dist = std::min(min_dist, dist);
                }
                int index = y * map->info.width + x;
                distance_map.data[index] = static_cast<int8_t>(std::min(100.0f, min_dist * 5));  // Scale for visualization
            }
        }

        distance_map_pub_.publish(distance_map);
        ROS_INFO("Distance map published");
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber map_sub_;
    ros::Publisher distance_map_pub_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "distance_map_generator");
    DistanceMapGenerator generator;
    ros::spin();
    return 0;
}
