#ifndef CUSTOM_ICP_H
#define CUSTOM_ICP_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <pcl/kdtree/kdtree_flann.h>

class CustomICP {
public:
    CustomICP();

    void setInputSource(pcl::PointCloud<pcl::PointXYZ>::Ptr source);
    void setInputTarget(pcl::PointCloud<pcl::PointXYZ>::Ptr target);

    void setMaxIterations(int max_iterations);
    void setTransformationEpsilon(double epsilon);
    void setMaxCorrespondenceDistance(double max_dist);

    bool align(Eigen::Matrix4f& final_transformation, const Eigen::Matrix4f& initial_guess);

    double getFitnessScore() const;
    int getIterations() const;

private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_;
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_;

    int max_iterations_;
    double epsilon_;
    double max_correspondence_distance_;
    double fitness_score_;
    int iterations_;

    void findCorrespondences(const Eigen::Matrix4f& transform, std::vector<int>& correspondences, std::vector<float>& distances);
    void estimateTransformation(const std::vector<int>& correspondences, Eigen::Matrix4f& transformation);
    void computeMeanAndCovariance(const std::vector<int>& correspondences, Eigen::Vector3f& mean_source, Eigen::Vector3f& mean_target, Eigen::Matrix3f& covariance);
    bool isConverged(const Eigen::Matrix4f& current, const Eigen::Matrix4f& previous);
};

#endif // CUSTOM_ICP_H

