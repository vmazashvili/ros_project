#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>

class LaserScanToPointCloud
{
public:
    LaserScanToPointCloud() : nh_("~")
    {
        scan_sub_ = nh_.subscribe("/scan", 1, &LaserScanToPointCloud::scanCallback, this);
        cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/laser_cloud", 1);
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
    {
        sensor_msgs::PointCloud2 cloud;
        
        try
        {
        	// Wait for the transform to be availabel
        	if(!tfListener_.waitForTransform(
        		scan->header.frame_id,
        		"base_link",
        		scan->header.stamp + ros::Duration().fromSec(scan->ranges.size()*scan->time_increment),
        		ros::Duration(1.0)))
        	{
        		ROS_WARN("Could not get transform, skipping scan");
        		return;
        	}
        	
            projector_.transformLaserScanToPointCloud("base_link", *scan, cloud, tfListener_);
        }
        catch (tf::TransformException& e)
        {
            ROS_ERROR("%s", e.what());
            return;
        }

        cloud_pub_.publish(cloud);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber scan_sub_;
    ros::Publisher cloud_pub_;
    tf::TransformListener tfListener_;
    laser_geometry::LaserProjection projector_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "laserscan_to_pointcloud");
    LaserScanToPointCloud converter;
    ros::spin();
    return 0;
}
