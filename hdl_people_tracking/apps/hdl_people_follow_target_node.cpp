#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <hdl_people_tracking_msgs/msg/track_array.hpp>

namespace hdl_people_tracking {

class HdlPeopleFollowTargetNode : public rclcpp::Node {
public:
  explicit HdlPeopleFollowTargetNode(const rclcpp::NodeOptions& options)
  : Node("hdl_people_follow_target_node", options)
  {
    base_frame_ = this->declare_parameter<std::string>("base_frame", "b2/base_link");
    track_frame_prefix_ = this->declare_parameter<std::string>("track_frame_prefix", "person_track_");
    transform_timeout_sec_ = this->declare_parameter<double>("transform_timeout_sec", 0.05);
    const std::string tracks_topic =
      this->declare_parameter<std::string>("tracks_topic", "hdl_people_tracking/tracks");
    const std::string follow_transform_topic =
      this->declare_parameter<std::string>("follow_transform_topic", "hdl_people_tracking/follow_person_transform");
    transform_timeout_sec_ = std::max(0.0, transform_timeout_sec_);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    follow_transform_pub_ = this->create_publisher<geometry_msgs::msg::TransformStamped>(
      follow_transform_topic, rclcpp::SensorDataQoS());

    tracks_sub_ = this->create_subscription<hdl_people_tracking_msgs::msg::TrackArray>(
      tracks_topic,
      10,
      std::bind(&HdlPeopleFollowTargetNode::tracks_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "Follow target selector publishing TransformStamped in '%s' from tracks in '%s'",
      base_frame_.c_str(),
      tracks_topic.c_str());
  }

private:
  void tracks_callback(const hdl_people_tracking_msgs::msg::TrackArray::ConstSharedPtr tracks_msg) {
    int selected_index = -1;
    geometry_msgs::msg::PoseStamped selected_pose;
    double selected_distance = std::numeric_limits<double>::max();

    for (size_t i = 0; i < tracks_msg->tracks.size(); i++) {
      geometry_msgs::msg::PoseStamped base_pose;
      if (!track_to_base_pose(tracks_msg->header, tracks_msg->tracks[i], base_pose)) {
        continue;
      }

      const double distance = std::hypot(base_pose.pose.position.x, base_pose.pose.position.y);
      if (distance < selected_distance) {
        selected_distance = distance;
        selected_index = static_cast<int>(i);
        selected_pose = base_pose;
      }
    }

    if (selected_index < 0) {
      return;
    }

    const auto& selected_track = tracks_msg->tracks[static_cast<size_t>(selected_index)];
    const auto transform = pose_to_transform(
      selected_pose,
      track_frame_prefix_ + std::to_string(selected_track.id));

    follow_transform_pub_->publish(transform);
    tf_broadcaster_->sendTransform(transform);
  }

  bool track_to_base_pose(
    const std_msgs::msg::Header& tracks_header,
    const hdl_people_tracking_msgs::msg::Track& track,
    geometry_msgs::msg::PoseStamped& base_pose) const
  {
    geometry_msgs::msg::PoseStamped track_pose;
    track_pose.header = tracks_header;
    track_pose.pose.position = track.pos;
    track_pose.pose.orientation.w = 1.0;

    if (track_pose.header.frame_id.empty()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Cannot convert selected track to %s: TrackArray header.frame_id is empty",
        base_frame_.c_str());
      return false;
    }

    if (track_pose.header.frame_id == base_frame_) {
      base_pose = track_pose;
      return true;
    }

    try {
      const auto transform = tf_buffer_->lookupTransform(
        base_frame_,
        track_pose.header.frame_id,
        track_pose.header.stamp,
        rclcpp::Duration::from_seconds(transform_timeout_sec_));
      tf2::doTransform(track_pose, base_pose, transform);
      return true;
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Cannot transform track from '%s' to '%s': %s",
        track_pose.header.frame_id.c_str(), base_frame_.c_str(), ex.what());
      return false;
    }
  }

  geometry_msgs::msg::TransformStamped pose_to_transform(
    const geometry_msgs::msg::PoseStamped& pose,
    const std::string& child_frame_id) const
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header = pose.header;
    transform.header.frame_id = base_frame_;
    transform.child_frame_id = child_frame_id;
    transform.transform.translation.x = pose.pose.position.x;
    transform.transform.translation.y = pose.pose.position.y;
    transform.transform.translation.z = pose.pose.position.z;
    transform.transform.rotation = pose.pose.orientation;
    if (
      transform.transform.rotation.x == 0.0 &&
      transform.transform.rotation.y == 0.0 &&
      transform.transform.rotation.z == 0.0 &&
      transform.transform.rotation.w == 0.0)
    {
      transform.transform.rotation.w = 1.0;
    }
    return transform;
  }

  rclcpp::Subscription<hdl_people_tracking_msgs::msg::TrackArray>::SharedPtr tracks_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr follow_transform_pub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::string base_frame_;
  std::string track_frame_prefix_;
  double transform_timeout_sec_;
};

}  // namespace hdl_people_tracking

RCLCPP_COMPONENTS_REGISTER_NODE(hdl_people_tracking::HdlPeopleFollowTargetNode)
