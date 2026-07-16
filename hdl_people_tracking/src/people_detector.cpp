#include <hdl_people_detection/people_detector.h>

#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cmath>
#include <limits>

#include <hdl_people_detection/marcel_people_detector.hpp>

namespace hdl_people_detection {

PeopleDetector::PeopleDetector(rclcpp::Node* node) {
  min_pts = node->declare_parameter<int>("cluster_min_pts", 10);
  max_pts = node->declare_parameter<int>("cluster_max_pts", 8192);
  min_size.x() = node->declare_parameter<double>("cluster_min_size_x", 0.2);
  min_size.y() = node->declare_parameter<double>("cluster_min_size_y", 0.2);
  min_size.z() = node->declare_parameter<double>("cluster_min_size_z", 0.3);
  max_size.x() = node->declare_parameter<double>("cluster_max_size_x", 1.0);
  max_size.y() = node->declare_parameter<double>("cluster_max_size_y", 1.0);
  max_size.z() = node->declare_parameter<double>("cluster_max_size_z", 2.0);
  euclidean_cluster_tolerance = static_cast<float>(node->declare_parameter<double>("euclidean_cluster_tolerance", 0.2));
  dpmeans_split_threshold = static_cast<float>(node->declare_parameter<double>("dpmeans_split_threshold", 0.45));
  enable_shape_filter = node->declare_parameter<bool>("enable_shape_filter", false);
  shape_min_width = static_cast<float>(node->declare_parameter<double>("shape_min_width", 0.18));
  shape_max_width = static_cast<float>(node->declare_parameter<double>("shape_max_width", 0.90));
  shape_min_depth = static_cast<float>(node->declare_parameter<double>("shape_min_depth", 0.02));
  shape_min_radius = static_cast<float>(node->declare_parameter<double>("shape_min_radius", 0.10));
  shape_max_radius = static_cast<float>(node->declare_parameter<double>("shape_max_radius", 0.45));
  shape_max_fit_rmse = static_cast<float>(node->declare_parameter<double>("shape_max_fit_rmse", 0.08));
  shape_max_linearity_ratio = static_cast<float>(node->declare_parameter<double>("shape_max_linearity_ratio", 80.0));

  if(euclidean_cluster_tolerance <= 0.0f) {
    RCLCPP_WARN(node->get_logger(), "euclidean_cluster_tolerance must be positive; using 0.2 m");
    euclidean_cluster_tolerance = 0.2f;
  }
  if(dpmeans_split_threshold <= 0.0f) {
    RCLCPP_WARN(node->get_logger(), "dpmeans_split_threshold must be positive; using 0.45 m");
    dpmeans_split_threshold = 0.45f;
  }
  if(shape_min_width < 0.0f || shape_max_width <= shape_min_width) {
    RCLCPP_WARN(node->get_logger(), "shape width limits are invalid; using 0.18-0.90 m");
    shape_min_width = 0.18f;
    shape_max_width = 0.90f;
  }
  if(shape_min_depth < 0.0f) {
    RCLCPP_WARN(node->get_logger(), "shape_min_depth must be non-negative; using 0.02 m");
    shape_min_depth = 0.02f;
  }
  if(shape_min_radius <= 0.0f || shape_max_radius <= shape_min_radius) {
    RCLCPP_WARN(node->get_logger(), "shape radius limits are invalid; using 0.10-0.45 m");
    shape_min_radius = 0.10f;
    shape_max_radius = 0.45f;
  }
  if(shape_max_fit_rmse <= 0.0f) {
    RCLCPP_WARN(node->get_logger(), "shape_max_fit_rmse must be positive; using 0.08 m");
    shape_max_fit_rmse = 0.08f;
  }
  if(shape_max_linearity_ratio <= 1.0f) {
    RCLCPP_WARN(node->get_logger(), "shape_max_linearity_ratio must be larger than 1; using 80.0");
    shape_max_linearity_ratio = 80.0f;
  }

  if(node->declare_parameter<bool>("enable_classification", false)) {
    try {
      std::string package_path = ament_index_cpp::get_package_share_directory("hdl_people_tracking");
      classifier.reset(new KidonoHumanClassifier(package_path + "/data/boost_kidono.model", package_path + "/data/boost_kidono.scale"));
    } catch (std::exception& e) {
      RCLCPP_ERROR(node->get_logger(), "failed to find package path: %s", e.what());
    }
  }
}

PeopleDetector::~PeopleDetector() {

}

std::vector<Cluster::Ptr> PeopleDetector::detect(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &cloud) const {
  if(!cloud || cloud->empty()) {
    return {};
  }

  MarcelPeopleDetector marcel(
    min_pts,
    max_pts,
    min_size,
    max_size,
    euclidean_cluster_tolerance,
    dpmeans_split_threshold);
  auto clusters = marcel.detect(cloud);

  for(auto& cluster : clusters) {
    const bool classified_as_human = !classifier || classifier->predict(cluster->cloud);
    const bool has_human_shape = !enable_shape_filter || passesShapeFilter(cluster->cloud);
    cluster->is_human = classified_as_human && has_human_shape;
  }

  return clusters;
}

bool PeopleDetector::passesShapeFilter(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud) const {
  if(!cloud || cloud->size() < 6) {
    return false;
  }

  Eigen::Vector2f mean = Eigen::Vector2f::Zero();
  size_t finite_points = 0;
  for(const auto& point : cloud->points) {
    if(!std::isfinite(point.x) || !std::isfinite(point.y)) {
      continue;
    }
    mean += Eigen::Vector2f(point.x, point.y);
    finite_points++;
  }

  if(finite_points < 6) {
    return false;
  }

  mean /= static_cast<float>(finite_points);
  const float mean_range = mean.norm();
  if(mean_range < 1e-3f) {
    return false;
  }

  const Eigen::Vector2f radial = mean / mean_range;
  const Eigen::Vector2f tangent(-radial.y(), radial.x());

  float min_tangent = std::numeric_limits<float>::max();
  float max_tangent = -std::numeric_limits<float>::max();
  float min_radial = std::numeric_limits<float>::max();
  float max_radial = -std::numeric_limits<float>::max();
  float sum_t = 0.0f;
  float sum_r = 0.0f;
  float sum_tt = 0.0f;
  float sum_rr = 0.0f;
  float sum_tr = 0.0f;

  for(const auto& point : cloud->points) {
    if(!std::isfinite(point.x) || !std::isfinite(point.y)) {
      continue;
    }
    const Eigen::Vector2f xy(point.x, point.y);
    const float t = tangent.dot(xy);
    const float r = radial.dot(xy);
    min_tangent = std::min(min_tangent, t);
    max_tangent = std::max(max_tangent, t);
    min_radial = std::min(min_radial, r);
    max_radial = std::max(max_radial, r);
    sum_t += t;
    sum_r += r;
    sum_tt += t * t;
    sum_rr += r * r;
    sum_tr += t * r;
  }

  const float n = static_cast<float>(finite_points);
  const float tangent_width = max_tangent - min_tangent;
  const float radial_depth = max_radial - min_radial;
  if(tangent_width < shape_min_width || tangent_width > shape_max_width || radial_depth < shape_min_depth) {
    return false;
  }

  const float mean_t = sum_t / n;
  const float mean_r = sum_r / n;
  const float cov_tt = std::max(0.0f, sum_tt / n - mean_t * mean_t);
  const float cov_rr = std::max(0.0f, sum_rr / n - mean_r * mean_r);
  const float cov_tr = sum_tr / n - mean_t * mean_r;
  const float trace = cov_tt + cov_rr;
  const float diff = cov_tt - cov_rr;
  const float discriminant = std::sqrt(std::max(0.0f, diff * diff + 4.0f * cov_tr * cov_tr));
  const float lambda_max = 0.5f * (trace + discriminant);
  const float lambda_min = 0.5f * (trace - discriminant);
  const float linearity_ratio = (lambda_max + 1e-6f) / (lambda_min + 1e-6f);
  if(linearity_ratio > shape_max_linearity_ratio) {
    return false;
  }

  Eigen::Matrix3f normal = Eigen::Matrix3f::Zero();
  Eigen::Vector3f rhs = Eigen::Vector3f::Zero();
  for(const auto& point : cloud->points) {
    if(!std::isfinite(point.x) || !std::isfinite(point.y)) {
      continue;
    }
    const Eigen::Vector2f xy(point.x, point.y);
    const float x = tangent.dot(xy) - mean_t;
    const float y = radial.dot(xy);
    const Eigen::Vector3f row(x * x, x, 1.0f);
    normal += row * row.transpose();
    rhs += row * y;
  }

  if(std::abs(normal.determinant()) < 1e-8f) {
    return false;
  }

  const Eigen::Vector3f curve = normal.ldlt().solve(rhs);
  const float curvature = curve.x();
  if(!std::isfinite(curvature) || curvature <= 0.0f) {
    return false;
  }

  const float radius = 1.0f / (2.0f * curvature);
  if(!std::isfinite(radius) || radius < shape_min_radius || radius > shape_max_radius) {
    return false;
  }

  float squared_error_sum = 0.0f;
  for(const auto& point : cloud->points) {
    if(!std::isfinite(point.x) || !std::isfinite(point.y)) {
      continue;
    }
    const Eigen::Vector2f xy(point.x, point.y);
    const float x = tangent.dot(xy) - mean_t;
    const float y = radial.dot(xy);
    const float predicted = curve.x() * x * x + curve.y() * x + curve.z();
    const float error = y - predicted;
    squared_error_sum += error * error;
  }

  const float rmse = std::sqrt(squared_error_sum / n);
  return std::isfinite(rmse) && rmse <= shape_max_fit_rmse;
}

}
