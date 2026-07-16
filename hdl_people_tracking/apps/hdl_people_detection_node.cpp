#include <algorithm>
#include <cmath>
#include <cctype>
#include <memory>
#include <string>
#include <vector>

#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <hdl_people_tracking_msgs/msg/cluster_array.hpp>

#include <hdl_people_detection/people_detector.h>

namespace hdl_people_tracking {

class HdlPeopleDetectionNode : public rclcpp::Node {
public:
  using PointT = pcl::PointXYZI;
  using Cloud = pcl::PointCloud<PointT>;

  explicit HdlPeopleDetectionNode(const rclcpp::NodeOptions& options)
  : Node("hdl_people_detection_node", options)
  {
    initialize_params();

    cropped_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("hdl_people_tracking/cropped_points", 5);
    cluster_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("hdl_people_tracking/cluster_points", 5);
    human_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("hdl_people_tracking/human_points", 5);
    detection_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("hdl_people_tracking/detection_markers", 5);
    clusters_pub_ = this->create_publisher<hdl_people_tracking_msgs::msg::ClusterArray>("hdl_people_tracking/clusters", 10);

    points_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "points",
      rclcpp::SensorDataQoS(),
      std::bind(&HdlPeopleDetectionNode::points_callback, this, std::placeholders::_1));
  }

private:
  struct Axis {
    int index;
    double sign;
  };

  static constexpr double kPi = 3.14159265358979323846;

  void initialize_params() {
    downsample_resolution_ = this->declare_parameter<double>("downsample_resolution", 0.1);
    min_detection_range_ = this->declare_parameter<double>("min_detection_range", 0.1);
    max_detection_range_ = this->declare_parameter<double>("max_detection_range", 5.0);
    min_detection_height_ = this->declare_parameter<double>("min_detection_height", 0.5);
    max_detection_height_ = this->declare_parameter<double>("max_detection_height", 1.8);
    front_fov_deg_ = this->declare_parameter<double>("front_fov_deg", 90.0);

    const std::string forward_axis = this->declare_parameter<std::string>("forward_axis", "x");
    const std::string lateral_axis = this->declare_parameter<std::string>("lateral_axis", "y");
    const std::string height_axis = this->declare_parameter<std::string>("height_axis", "z");
    forward_axis_ = parse_axis(forward_axis, Axis{0, 1.0}, "forward_axis");
    lateral_axis_ = parse_axis(lateral_axis, Axis{1, 1.0}, "lateral_axis");
    height_axis_ = parse_axis(height_axis, Axis{2, 1.0}, "height_axis");
    if (forward_axis_.index == lateral_axis_.index) {
      RCLCPP_WARN(
        this->get_logger(),
        "forward_axis and lateral_axis refer to the same coordinate; using lateral_axis='y'");
      lateral_axis_ = Axis{1, 1.0};
    }
    if (height_axis_.index == forward_axis_.index || height_axis_.index == lateral_axis_.index) {
      RCLCPP_WARN(this->get_logger(), "height_axis overlaps with a crop axis; using the remaining coordinate");
      for (int index = 0; index < 3; index++) {
        if (index != forward_axis_.index && index != lateral_axis_.index) {
          height_axis_ = Axis{index, 1.0};
          break;
        }
      }
    }

    if (downsample_resolution_ < 0.0) {
      RCLCPP_WARN(this->get_logger(), "downsample_resolution must be non-negative; disabling downsampling");
      downsample_resolution_ = 0.0;
    }
    min_detection_range_ = std::max(0.0, min_detection_range_);
    if (max_detection_range_ <= min_detection_range_) {
      RCLCPP_WARN(
        this->get_logger(),
        "max_detection_range must be larger than min_detection_range; using 5.0 m");
      max_detection_range_ = 5.0;
    }
    if (max_detection_height_ <= min_detection_height_) {
      RCLCPP_WARN(
        this->get_logger(),
        "max_detection_height must be larger than min_detection_height; using 0.5-1.8 m");
      min_detection_height_ = 0.5;
      max_detection_height_ = 1.8;
    }

    front_fov_deg_ = std::max(1.0, std::min(front_fov_deg_, 179.0));
    half_fov_tangent_ = std::tan((front_fov_deg_ * kPi / 180.0) * 0.5);

    RCLCPP_INFO(
      this->get_logger(),
      "People detector crop: %.1f deg FOV, %.2f-%.2f m range, %.2f-%.2f m height, forward=%s, lateral=%s, height=%s",
      front_fov_deg_, min_detection_range_, max_detection_range_,
      min_detection_height_, max_detection_height_,
      forward_axis.c_str(), lateral_axis.c_str(), height_axis.c_str());

    detector_ = std::make_unique<hdl_people_detection::PeopleDetector>(this);
  }

  Axis parse_axis(const std::string& value, const Axis& fallback, const char* parameter_name) const {
    if (value.empty()) {
      RCLCPP_WARN(this->get_logger(), "%s is empty; using default", parameter_name);
      return fallback;
    }

    double sign = 1.0;
    size_t char_index = 0;
    if (value[0] == '-') {
      sign = -1.0;
      char_index = 1;
    } else if (value[0] == '+') {
      char_index = 1;
    }

    if (char_index >= value.size()) {
      RCLCPP_WARN(this->get_logger(), "%s='%s' is invalid; using default", parameter_name, value.c_str());
      return fallback;
    }

    const char axis = static_cast<char>(std::tolower(value[char_index]));
    if (axis == 'x') {
      return Axis{0, sign};
    }
    if (axis == 'y') {
      return Axis{1, sign};
    }
    if (axis == 'z') {
      return Axis{2, sign};
    }

    RCLCPP_WARN(this->get_logger(), "%s='%s' is invalid; using default", parameter_name, value.c_str());
    return fallback;
  }

  static double coordinate(const PointT& point, const Axis& axis) {
    if (axis.index == 0) {
      return axis.sign * point.x;
    }
    if (axis.index == 1) {
      return axis.sign * point.y;
    }
    return axis.sign * point.z;
  }

  void points_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr points_msg) {
    Cloud::Ptr cloud(new Cloud());
    pcl::fromROSMsg(*points_msg, *cloud);
    if (cloud->empty()) {
      publish_empty(points_msg->header);
      return;
    }

    Cloud::Ptr cropped = crop_cloud(cloud);
    if (cropped->empty()) {
      publish_empty(points_msg->header);
      return;
    }

    Cloud::Ptr filtered = downsample(cropped);
    auto clusters = detector_->detect(filtered);

    publish_msgs(points_msg->header, filtered, clusters);
  }

  Cloud::Ptr crop_cloud(const Cloud::ConstPtr& cloud) const {
    Cloud::Ptr cropped(new Cloud());
    cropped->reserve(cloud->size());

    const double min_range_sq = min_detection_range_ * min_detection_range_;
    const double max_range_sq = max_detection_range_ * max_detection_range_;

    for (const auto& point : cloud->points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        continue;
      }

      const double forward = coordinate(point, forward_axis_);
      const double lateral = coordinate(point, lateral_axis_);
      const double height = coordinate(point, height_axis_);
      if (forward <= 0.0) {
        continue;
      }
      if (height < min_detection_height_ || height > max_detection_height_) {
        continue;
      }

      const double range_sq = forward * forward + lateral * lateral;
      if (range_sq < min_range_sq || range_sq > max_range_sq) {
        continue;
      }

      if (std::abs(lateral) > forward * half_fov_tangent_) {
        continue;
      }

      cropped->push_back(point);
    }

    cropped->header = cloud->header;
    cropped->width = static_cast<uint32_t>(cropped->size());
    cropped->height = 1;
    cropped->is_dense = false;
    return cropped;
  }

  Cloud::Ptr downsample(const Cloud::ConstPtr& cloud) const {
    if (downsample_resolution_ <= 0.0) {
      Cloud::Ptr copy(new Cloud(*cloud));
      return copy;
    }

    Cloud::Ptr downsampled(new Cloud());
    pcl::VoxelGrid<PointT> voxelgrid;
    voxelgrid.setLeafSize(downsample_resolution_, downsample_resolution_, downsample_resolution_);
    voxelgrid.setInputCloud(cloud);
    voxelgrid.filter(*downsampled);
    downsampled->header = cloud->header;
    return downsampled;
  }

  void publish_empty(const std_msgs::msg::Header& header) const {
    Cloud::Ptr empty(new Cloud());
    empty->width = 0;
    empty->height = 1;
    empty->is_dense = false;

    publish_cloud(header, empty, cropped_points_pub_);
    publish_cloud(header, empty, cluster_points_pub_);
    publish_cloud(header, empty, human_points_pub_);

    hdl_people_tracking_msgs::msg::ClusterArray clusters_msg;
    clusters_msg.header = header;
    clusters_pub_->publish(clusters_msg);
    detection_markers_pub_->publish(visualization_msgs::msg::MarkerArray());
  }

  void publish_msgs(
    const std_msgs::msg::Header& header,
    const Cloud::ConstPtr& filtered,
    const std::vector<hdl_people_detection::Cluster::Ptr>& clusters) const
  {
    hdl_people_tracking_msgs::msg::ClusterArray clusters_msg;
    clusters_msg.header = header;
    clusters_msg.clusters.resize(clusters.size());

    Cloud::Ptr person_points(new Cloud());

    for (size_t i = 0; i < clusters.size(); i++) {
      const auto& cluster = clusters[i];
      auto& cluster_msg = clusters_msg.clusters[i];

      cluster_msg.is_human = cluster->is_human;
      cluster_msg.min_pt.x = cluster->min_pt.x();
      cluster_msg.min_pt.y = cluster->min_pt.y();
      cluster_msg.min_pt.z = cluster->min_pt.z();
      cluster_msg.max_pt.x = cluster->max_pt.x();
      cluster_msg.max_pt.y = cluster->max_pt.y();
      cluster_msg.max_pt.z = cluster->max_pt.z();
      cluster_msg.size.x = cluster->size.x();
      cluster_msg.size.y = cluster->size.y();
      cluster_msg.size.z = cluster->size.z();
      cluster_msg.centroid.x = cluster->centroid.x();
      cluster_msg.centroid.y = cluster->centroid.y();
      cluster_msg.centroid.z = cluster->centroid.z();

      if (cluster->is_human) {
        std::copy(cluster->cloud->begin(), cluster->cloud->end(), std::back_inserter(person_points->points));
      }
    }

    clusters_pub_->publish(clusters_msg);
    publish_cloud(header, filtered, cropped_points_pub_);
    publish_cloud(header, finalize_cloud(person_points), cluster_points_pub_);
    publish_cloud(header, finalize_cloud(person_points), human_points_pub_);
    detection_markers_pub_->publish(create_markers(header, clusters));
  }

  Cloud::Ptr finalize_cloud(const Cloud::Ptr& cloud) const {
    cloud->width = static_cast<uint32_t>(cloud->size());
    cloud->height = 1;
    cloud->is_dense = false;
    return cloud;
  }

  void publish_cloud(
    const std_msgs::msg::Header& header,
    const Cloud::ConstPtr& cloud,
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& publisher) const
  {
    sensor_msgs::msg::PointCloud2 ros_cloud;
    pcl::toROSMsg(*cloud, ros_cloud);
    ros_cloud.header = header;
    publisher->publish(ros_cloud);
  }

  visualization_msgs::msg::MarkerArray create_markers(
    const std_msgs::msg::Header& header,
    const std::vector<hdl_people_detection::Cluster::Ptr>& clusters) const
  {
    visualization_msgs::msg::MarkerArray markers;

    for (size_t i = 0; i < clusters.size(); i++) {
      if (!clusters[i]->is_human) {
        continue;
      }

      visualization_msgs::msg::Marker cluster_marker;
      cluster_marker.header = header;
      cluster_marker.action = visualization_msgs::msg::Marker::ADD;
      cluster_marker.lifetime = rclcpp::Duration::from_seconds(0.5);
      cluster_marker.ns = "detection_" + std::to_string(i);
      cluster_marker.id = static_cast<int>(i);
      cluster_marker.type = visualization_msgs::msg::Marker::CUBE;
      cluster_marker.pose.position.x = clusters[i]->centroid.x();
      cluster_marker.pose.position.y = clusters[i]->centroid.y();
      cluster_marker.pose.position.z = clusters[i]->centroid.z();
      cluster_marker.pose.orientation.w = 1.0;
      cluster_marker.color.b = 1.0;
      cluster_marker.color.a = 0.4;
      cluster_marker.scale.x = std::max(0.05f, clusters[i]->size.x());
      cluster_marker.scale.y = std::max(0.05f, clusters[i]->size.y());
      cluster_marker.scale.z = std::max(0.05f, clusters[i]->size.z());
      markers.markers.push_back(cluster_marker);
    }

    return markers;
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cropped_points_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cluster_points_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr human_points_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr detection_markers_pub_;
  rclcpp::Publisher<hdl_people_tracking_msgs::msg::ClusterArray>::SharedPtr clusters_pub_;

  double downsample_resolution_;
  double min_detection_range_;
  double max_detection_range_;
  double min_detection_height_;
  double max_detection_height_;
  double front_fov_deg_;
  double half_fov_tangent_;
  Axis forward_axis_;
  Axis lateral_axis_;
  Axis height_axis_;

  std::unique_ptr<hdl_people_detection::PeopleDetector> detector_;
};

}  // namespace hdl_people_tracking

RCLCPP_COMPONENTS_REGISTER_NODE(hdl_people_tracking::HdlPeopleDetectionNode)
