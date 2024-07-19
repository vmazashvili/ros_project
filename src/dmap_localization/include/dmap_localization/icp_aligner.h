#ifndef ICP_ALIGNER_H
#define ICP_ALIGNER_H

#include <pcl/registration/icp.h>
#include <pcl/point_cloud.h>
#include <geometry_msgs/Pose.h>

class ICPAligner {
public:
    ICPAligner();
    void configure(double max_correspondence_distance, int max_iterations, double transformation_epsilon);
    bool align(const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud,
               const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud,
               Eigen::Matrix4f& transformation);

private:
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp_;
};

#endif  // ICP_ALIGNER_H

