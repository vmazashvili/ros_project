#include "dmap_localization/custom_icp.h"
#include <numeric>
#include <pcl/kdtree/kdtree_flann.h>
#include <Eigen/SVD>

CustomICP::CustomICP() : max_iterations_(50), epsilon_(1e-8), max_correspondence_distance_(0.1) {}

void CustomICP::setInputSource(pcl::PointCloud<pcl::PointXYZ>::Ptr source) {
    source_cloud_ = source;
}

void CustomICP::setInputTarget(pcl::PointCloud<pcl::PointXYZ>::Ptr target) {
    target_cloud_ = target;
    kdtree_.setInputCloud(target_cloud_);
}

void CustomICP::setMaxIterations(int max_iterations) { max_iterations_ = max_iterations; }
void CustomICP::setTransformationEpsilon(double epsilon) { epsilon_ = epsilon; }
void CustomICP::setMaxCorrespondenceDistance(double max_dist) { max_correspondence_distance_ = max_dist; }

bool CustomICP::align(Eigen::Matrix4f& final_transformation, const Eigen::Matrix4f& initial_guess) {
    Eigen::Matrix4f current_transform = initial_guess;
    Eigen::Matrix4f previous_transform;

    for (int iteration = 0; iteration < max_iterations_; ++iteration) {
        std::vector<int> correspondences;
        std::vector<float> distances;
        findCorrespondences(current_transform, correspondences, distances);

        Eigen::Matrix4f transformation;
        estimateTransformation(correspondences, transformation);

        current_transform = transformation * current_transform;

        if (iteration > 0 && isConverged(current_transform, previous_transform)) {
            final_transformation = current_transform;
            return true;
        }

        previous_transform = current_transform;
    }

    final_transformation = current_transform;
    return false;
}

double CustomICP::getFitnessScore() const { return fitness_score_; }
int CustomICP::getIterations() const { return iterations_; }

void CustomICP::findCorrespondences(const Eigen::Matrix4f& transform, std::vector<int>& correspondences, std::vector<float>& distances) {
    correspondences.clear();
    distances.clear();

    for (const auto& point : source_cloud_->points) {
        Eigen::Vector4f transformed_point = transform * Eigen::Vector4f(point.x, point.y, point.z, 1.0f);
        pcl::PointXYZ search_point;
        search_point.x = transformed_point[0];
        search_point.y = transformed_point[1];
        search_point.z = transformed_point[2];

        std::vector<int> neighbor_indices(1);
        std::vector<float> neighbor_distances(1);

        if (kdtree_.nearestKSearch(search_point, 1, neighbor_indices, neighbor_distances) > 0) {
            if (neighbor_distances[0] <= max_correspondence_distance_ * max_correspondence_distance_) {
                correspondences.push_back(neighbor_indices[0]);
                distances.push_back(neighbor_distances[0]);
            }
        }
    }

    fitness_score_ = std::accumulate(distances.begin(), distances.end(), 0.0) / distances.size();
}

void CustomICP::estimateTransformation(const std::vector<int>& correspondences, Eigen::Matrix4f& transformation) {
    Eigen::Matrix3f covariance;
    Eigen::Vector3f mean_source, mean_target;
    computeMeanAndCovariance(correspondences, mean_source, mean_target, covariance);

    Eigen::JacobiSVD<Eigen::Matrix3f> svd(covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3f rotation = svd.matrixU() * svd.matrixV().transpose();

    if (rotation.determinant() < 0) {
        rotation = svd.matrixU() * Eigen::Vector3f(1, 1, -1).asDiagonal() * svd.matrixV().transpose();
    }

    Eigen::Vector3f translation = mean_target - rotation * mean_source;

    transformation.setIdentity();
    transformation.block<3, 3>(0, 0) = rotation;
    transformation.block<3, 1>(0, 3) = translation;
}

void CustomICP::computeMeanAndCovariance(const std::vector<int>& correspondences, Eigen::Vector3f& mean_source, Eigen::Vector3f& mean_target, Eigen::Matrix3f& covariance) {
    mean_source.setZero();
    mean_target.setZero();
    covariance.setZero();

    for (size_t i = 0; i < correspondences.size(); ++i) {
        const auto& source_point = source_cloud_->points[i];
        const auto& target_point = target_cloud_->points[correspondences[i]];

        mean_source += Eigen::Vector3f(source_point.x, source_point.y, source_point.z);
        mean_target += Eigen::Vector3f(target_point.x, target_point.y, target_point.z);
    }

    mean_source /= correspondences.size();
    mean_target /= correspondences.size();

    for (size_t i = 0; i < correspondences.size(); ++i) {
        const auto& source_point = source_cloud_->points[i];
        const auto& target_point = target_cloud_->points[correspondences[i]];

        Eigen::Vector3f source_demean = Eigen::Vector3f(source_point.x, source_point.y, source_point.z) - mean_source;
        Eigen::Vector3f target_demean = Eigen::Vector3f(target_point.x, target_point.y, target_point.z) - mean_target;

        covariance += source_demean * target_demean.transpose();
    }

    covariance /= correspondences.size();
}

bool CustomICP::isConverged(const Eigen::Matrix4f& current, const Eigen::Matrix4f& previous) {
    double rot_epsilon = 2 * epsilon_;
    double trans_epsilon = epsilon_;

    Eigen::Matrix3f R = current.block<3, 3>(0, 0);
    Eigen::Matrix3f R_prev = previous.block<3, 3>(0, 0);
    Eigen::Vector3f t = current.block<3, 1>(0, 3);
    Eigen::Vector3f t_prev = previous.block<3, 1>(0, 3);

    double rot_error = (R * R_prev.transpose()).diagonal().array().abs().sum();
    double trans_error = (t - t_prev).norm();

    return (rot_error < rot_epsilon && trans_error < trans_epsilon);
}

