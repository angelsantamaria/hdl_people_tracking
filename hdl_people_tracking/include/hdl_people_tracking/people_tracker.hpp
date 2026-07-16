#ifndef PEOPLE_TRACKER_HPP
#define PEOPLE_TRACKER_HPP

#include <algorithm>
#include <cmath>
#include <limits>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <unordered_map>
#include <Eigen/Dense>
#include <boost/optional.hpp>
#include <opencv2/opencv.hpp>

#include <kkl/alg/data_association.hpp>
#include <kkl/alg/nearest_neighbor_association.hpp>

#include <hdl_people_tracking_msgs/msg/cluster.hpp>
#include <hdl_people_tracking/kalman_tracker.hpp>

namespace kkl {
  namespace alg {

/**
 * @brief definition of the distance between tracker and observation for data association
 */
template<>
boost::optional<double> distance(const std::shared_ptr<hdl_people_tracking::KalmanTracker>& tracker, const hdl_people_tracking_msgs::msg::Cluster& observation) {
  Eigen::Vector3d pos(observation.centroid.x, observation.centroid.y, observation.centroid.z);
  double sq_mahalanobis = tracker->squaredMahalanobisDistance(pos);

  // gating
  if(sq_mahalanobis > pow(3.0, 2) || (tracker->position() - pos).norm() > 1.5) {
    return boost::none;
  }
  return -kkl::math::gaussianProbMul(tracker->position(), tracker->positionCov(), pos);
}
  }
}

namespace hdl_people_tracking {

/**
 * @brief People tracker
 */
class PeopleTracker {
public:
  PeopleTracker(rclcpp::Node* node) {
    id_gen = 0;
    human_radius = node->declare_parameter<double>("human_radius", 0.4);
    remove_trace_thresh = node->declare_parameter<double>("remove_trace_thresh", 1.0);
    single_target_mode = node->declare_parameter<bool>("track_single_target_mode", false);
    init_centerline_only = node->declare_parameter<bool>("track_init_centerline_only", false);
    init_centerline_angle_deg = node->declare_parameter<double>("track_init_centerline_angle_deg", 5.0);
    init_min_range = node->declare_parameter<double>("track_init_min_range", 0.0);
    init_max_range = node->declare_parameter<double>("track_init_max_range", 0.0);
    init_preferred_range = node->declare_parameter<double>("track_init_preferred_range", 3.0);
    association_max_gap_sec = node->declare_parameter<double>("track_association_max_gap_sec", 0.5);
    association_max_angle_delta_deg = node->declare_parameter<double>("track_association_max_angle_delta_deg", 15.0);
    if(init_centerline_angle_deg < 0.0) {
      RCLCPP_WARN(node->get_logger(), "track_init_centerline_angle_deg must be non-negative; using 5.0 deg");
      init_centerline_angle_deg = 5.0;
    }
    if(init_min_range < 0.0) {
      RCLCPP_WARN(node->get_logger(), "track_init_min_range must be non-negative; using 0.0 m");
      init_min_range = 0.0;
    }
    if(init_max_range < 0.0) {
      RCLCPP_WARN(node->get_logger(), "track_init_max_range must be non-negative; disabling upper range gate");
      init_max_range = 0.0;
    }
    if(init_max_range > 0.0 && init_max_range <= init_min_range) {
      RCLCPP_WARN(node->get_logger(), "track_init_max_range must be larger than track_init_min_range; disabling upper range gate");
      init_max_range = 0.0;
    }
    if(init_preferred_range < init_min_range) {
      init_preferred_range = init_min_range;
    }
    if(init_max_range > 0.0 && init_preferred_range > init_max_range) {
      init_preferred_range = init_max_range;
    }
    if(association_max_gap_sec < 0.0) {
      RCLCPP_WARN(node->get_logger(), "track_association_max_gap_sec must be non-negative; using 0.5 s");
      association_max_gap_sec = 0.5;
    }
    if(association_max_angle_delta_deg < 0.0) {
      RCLCPP_WARN(node->get_logger(), "track_association_max_angle_delta_deg must be non-negative; using 15.0 deg");
      association_max_angle_delta_deg = 15.0;
    }

    data_association.reset(new kkl::alg::NearestNeighborAssociation<KalmanTracker::Ptr, hdl_people_tracking_msgs::msg::Cluster>());
//    data_association.reset(new kkl::alg::GlobalNearestNeighborAssociation<KalmanTracker::Ptr, VisualDetection>());
  }

  /**
   * @brief predict people states
   * @param time  current time
   */
  void predict(const rclcpp::Time& time) {
    for(auto& person : people) {
      person->predict(time);
    }
  }

  /**
   * @brief correct people states
   * @param time          current time
   * @param detections    detections
   */
  void correct(const rclcpp::Time& time, const std::vector<hdl_people_tracking_msgs::msg::Cluster>& detections) {
    // data association
    std::vector<bool> associated(detections.size(), false);
    auto associations = data_association->associate(people, detections);
    for(const auto& assoc : associations) {
      const auto& observation = detections[assoc.observation].centroid;
      Eigen::Vector3d observation_pos(observation.x, observation.y, observation.z);
      if(!passesAssociationContinuity(people[assoc.tracker], observation_pos, time)) {
        continue;
      }

      associated[assoc.observation] = true;
      people[assoc.tracker]->correct(time, observation_pos, detections[assoc.observation]);
    }

    // generate new tracks
    if(single_target_mode) {
      if(people.empty()) {
        const int detection_index = selectBestInitDetection(detections, associated);
        if(detection_index >= 0) {
          const auto& observation = detections[detection_index].centroid;
          Eigen::Vector3d observation_pos(observation.x, observation.y, observation.z);
          KalmanTracker::Ptr tracker(new KalmanTracker(id_gen++, time, observation_pos));
          people.push_back(tracker);
        }
      }
    } else {
      for(size_t i=0; i<detections.size(); i++) {
        if(!associated[i]) {
          const auto& observation = detections[i].centroid;
          Eigen::Vector3d observation_pos(observation.x, observation.y, observation.z);

          if(!passesTrackInitGate(observation_pos) || isCloseToExistingTrack(observation_pos)) {
            continue;
          }

          KalmanTracker::Ptr tracker(new KalmanTracker(id_gen++, time, observation_pos));
          people.push_back(tracker);
        }
      }
    }

    // remove tracks with large covariance
    auto remove_loc = std::partition(people.begin(), people.end(), [&](const KalmanTracker::Ptr& tracker) {
      return tracker->positionCov().trace() < remove_trace_thresh;
    });
    removed_people.clear();
    std::copy(remove_loc, people.end(), std::back_inserter(removed_people));
    people.erase(remove_loc, people.end());
    if(single_target_mode && people.size() > 1) {
      std::copy(people.begin() + 1, people.end(), std::back_inserter(removed_people));
      people.erase(people.begin() + 1, people.end());
    }
  }

private:
  bool isInsideInitCenterline(const Eigen::Vector3d& position) const {
    if(position.x() <= 0.0) {
      return false;
    }

    constexpr double kPi = 3.14159265358979323846;
    const double angle_deg = std::abs(std::atan2(position.y(), position.x())) * 180.0 / kPi;
    return angle_deg <= init_centerline_angle_deg;
  }

  bool passesTrackInitGate(const Eigen::Vector3d& position) const {
    if(init_centerline_only && !isInsideInitCenterline(position)) {
      return false;
    }

    const double range = position.head<2>().norm();
    if(range < init_min_range) {
      return false;
    }
    if(init_max_range > 0.0 && range > init_max_range) {
      return false;
    }

    return true;
  }

  bool isCloseToExistingTrack(const Eigen::Vector3d& position) const {
    for(const auto& person : people) {
      if((person->position() - position).norm() < human_radius * 2.0) {
        return true;
      }
    }
    return false;
  }

  double trackInitScore(const Eigen::Vector3d& position) const {
    constexpr double kPi = 3.14159265358979323846;
    const double range = position.head<2>().norm();
    const double range_score = std::abs(range - init_preferred_range);
    const double angle_score = std::abs(std::atan2(position.y(), position.x())) * 180.0 / kPi;
    return range_score + 0.02 * angle_score;
  }

  int selectBestInitDetection(
    const std::vector<hdl_people_tracking_msgs::msg::Cluster>& detections,
    const std::vector<bool>& associated) const
  {
    int best_index = -1;
    double best_score = std::numeric_limits<double>::max();
    for(size_t i=0; i<detections.size(); i++) {
      if(associated[i]) {
        continue;
      }

      const auto& observation = detections[i].centroid;
      Eigen::Vector3d observation_pos(observation.x, observation.y, observation.z);
      if(!passesTrackInitGate(observation_pos) || isCloseToExistingTrack(observation_pos)) {
        continue;
      }

      const double score = trackInitScore(observation_pos);
      if(score < best_score) {
        best_score = score;
        best_index = static_cast<int>(i);
      }
    }

    return best_index;
  }

  bool passesAssociationContinuity(
    const KalmanTracker::Ptr& tracker,
    const Eigen::Vector3d& observation,
    const rclcpp::Time& time) const
  {
    if(association_max_gap_sec > 0.0 &&
      (time - tracker->lastCorrectionTime()).seconds() > association_max_gap_sec)
    {
      return false;
    }

    if(association_max_angle_delta_deg <= 0.0) {
      return true;
    }

    const Eigen::Vector3d predicted = tracker->position();
    if(predicted.head<2>().norm() < 1e-3 || observation.head<2>().norm() < 1e-3) {
      return false;
    }

    constexpr double kPi = 3.14159265358979323846;
    const double predicted_angle = std::atan2(predicted.y(), predicted.x());
    const double observation_angle = std::atan2(observation.y(), observation.x());
    double angle_delta = std::abs(observation_angle - predicted_angle);
    if(angle_delta > kPi) {
      angle_delta = 2.0 * kPi - angle_delta;
    }

    return angle_delta * 180.0 / kPi <= association_max_angle_delta_deg;
  }

public:
  long id_gen;                  // track ID which will be assigned to the next new track
  double human_radius;          // new tracks must be far from existing tracks than this value
  double remove_trace_thresh;   // tracks with larger covariance trace than this will be removed
  bool single_target_mode;
  bool init_centerline_only;     // new tracks are initialized only near the forward ray
  double init_centerline_angle_deg;
  double init_min_range;
  double init_max_range;
  double init_preferred_range;
  double association_max_gap_sec;
  double association_max_angle_delta_deg;

  std::vector<KalmanTracker::Ptr> people;
  std::vector<KalmanTracker::Ptr> removed_people;
  std::unique_ptr<kkl::alg::DataAssociation<KalmanTracker::Ptr, hdl_people_tracking_msgs::msg::Cluster>> data_association;
};

}

#endif // PEOPLE_TRACKER_HPP
