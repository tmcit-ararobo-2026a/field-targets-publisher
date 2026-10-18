// Copyright 2026 tmcit-ararobo-2026a
// SPDX-License-Identifier: MIT
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <nlohmann/json.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <chrono>
#include <cmath>
#include <fstream>
#include <set>
#include <stdexcept>
#include <vector>

#include "field_target_pub/msg/target_set.hpp"

namespace field_target_pub {
/** @brief Approved fixed world targets, aiming points and stop poses without coordinate inference. */
class FieldTargetNode : public rclcpp::Node {
public:
  /** @brief Read an explicit team/map profile; the default template remains invalid. */
  FieldTargetNode() : Node("field_target_node") {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    const auto file = declare_parameter<std::string>("config_file", "", descriptor);
    expected_map_id_ = declare_parameter<std::string>("expected_map_id", "", descriptor);
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link", descriptor);
    pose_timeout_s_ = declare_parameter<double>("pose_timeout_s", 0.5, descriptor);
    if (!std::isfinite(pose_timeout_s_) || pose_timeout_s_ <= 0 || base_frame_.empty()) {
      throw std::invalid_argument("Invalid pose deadline/frame");
    }
    publisher_ = create_publisher<msg::TargetSet>("/field/targets", rclcpp::QoS(1).durability_volatile());
    buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    listener_ = std::make_unique<tf2_ros::TransformListener>(*buffer_);
    broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    set_.header.frame_id = "map";
    set_.reason = "Approved team/map/target configuration is missing";
    if (!file.empty()) {
      load(file);
    }
    timer_ = create_wall_timer(std::chrono::milliseconds(20), [this]() { publish(); });
  }
private:
  /** @brief Read a finite numeric vector of an exact configured dimension. */
  std::vector<double> vector(const nlohmann::json & values, std::size_t size) const {
    if (!values.is_array() || values.size() != size) {
      throw std::invalid_argument("Target coordinate vector has wrong size");
    }
    std::vector<double> output;
    for (const auto & value : values) {
      const double number = value.get<double>();
      if (!std::isfinite(number)) {
        throw std::invalid_argument("Target coordinate must be finite");
      }
      output.push_back(number);
    }
    return output;
  }
  /** @brief Validate every fixed ID and distinguish centers, aiming points and stop poses. */
  void load(const std::string & file) {
    std::ifstream stream(file);
    if (!stream) {
      throw std::invalid_argument("Cannot open target configuration");
    }
    nlohmann::json config;
    stream >> config;
    if (config.at("schema_version").get<int>() != 1) {
      throw std::invalid_argument("Unsupported target schema");
    }
    if (!config.at("approved").get<bool>()) {
      return;  // Unapproved coordinates are not emitted as targets or static TF.
    }
    set_.team = config.at("team").get<std::string>();
    set_.map_id = config.at("map_id").get<std::string>();
    set_.header.frame_id = config.at("map_frame").get<std::string>();
    set_.revision = config.at("revision").get<uint64_t>();
    if ((set_.team != "red" && set_.team != "blue") || set_.map_id.empty() ||
      set_.header.frame_id.empty() || set_.header.frame_id == base_frame_ || set_.revision == 0)
    {
      throw std::invalid_argument("Explicit red/blue team, map and revision are required");
    }
    if (!expected_map_id_.empty() && expected_map_id_ != set_.map_id) {
      set_.reason = "Target profile does not match the expected map";
      return;
    }
    std::set<int> identifiers;
    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    for (const auto & entry : config.at("targets")) {
      const int identifier = entry.at("target_id").get<int>();
      if (identifier < 0 || identifier > 6 || !identifiers.insert(identifier).second) {
        throw std::invalid_argument("Every fixed target ID must appear exactly once");
      }
      msg::FieldTarget target;
      target.target_id = static_cast<uint8_t>(identifier);
      target.label = entry.at("label").get<std::string>();
      const auto stop = vector(entry.at("stop_pose"), 4);  // x,y,z,yaw in map.
      if (target.label.empty() || stop[2] != 0) {
        throw std::invalid_argument("A labeled stop pose on the 2D navigation plane is required");
      }
      target.stop_pose.header.frame_id = set_.header.frame_id;
      target.stop_pose.pose.position.x = stop[0];
      target.stop_pose.pose.position.y = stop[1];
      target.stop_pose.pose.orientation.z = std::sin(stop[3] * 0.5);
      target.stop_pose.pose.orientation.w = std::cos(stop[3] * 0.5);
      target.stop_pose_valid = true;
      if (identifier != 0) {
        const auto aim = vector(entry.at("aim_point"), 3);
        const auto center = vector(entry.at("object_center"), 3);
        target.aim_point.header.frame_id = set_.header.frame_id;
        target.aim_point.point.x = aim[0];
        target.aim_point.point.y = aim[1];
        target.aim_point.point.z = aim[2];
        target.aim_valid = true;
        geometry_msgs::msg::TransformStamped transform;
        transform.header.frame_id = set_.header.frame_id;
        if (identifier <= 3) {
          transform.child_frame_id = "bucket_" + std::to_string(identifier) + "_center";
        } else if (identifier <= 5) {
          transform.child_frame_id = "desk_" + std::to_string(identifier - 3) + "_center";
        } else {
          transform.child_frame_id = "flag_1_center";
        }
        transform.transform.translation.x = center[0];
        transform.transform.translation.y = center[1];
        transform.transform.translation.z = center[2];
        transform.transform.rotation.w = 1;
        transforms.push_back(transform);
      }
      // STM32 sign/rounding/invalid-angle contract is unresolved: never invent an int8 angle.
      target.angle_valid = false;
      set_.targets.push_back(target);
    }
    std::set<std::string> frames;
    for (const auto & transform : transforms) {
      frames.insert(transform.child_frame_id);
    }
    for (const auto & object : config.value("static_objects", nlohmann::json::array())) {
      const auto child = object.at("frame_id").get<std::string>();
      const bool recognized = child.rfind("bucket_", 0) == 0 || child.rfind("desk_", 0) == 0 ||
                              child.rfind("flag_", 0) == 0;
      if (!recognized || !frames.insert(child).second) {
        throw std::invalid_argument("Static object frame must have a unique bucket_/desk_/flag_ prefix");
      }
      const auto center = vector(object.at("object_center"), 3);
      geometry_msgs::msg::TransformStamped transform;
      transform.header.frame_id = set_.header.frame_id;
      transform.child_frame_id = child;
      transform.transform.translation.x = center[0];
      transform.transform.translation.y = center[1];
      transform.transform.translation.z = center[2];
      transform.transform.rotation.w = 1;
      transforms.push_back(transform);
    }
    if (identifiers.size() != 7) {
      throw std::invalid_argument("Target profile must include all fixed IDs 0..6");
    }
    set_.valid = true;
    set_.reason = "Approved profile; STM32 angle conversion remains unresolved";
    for (auto & transform : transforms) {
      transform.header.stamp = now();
    }
    broadcaster_->sendTransform(transforms);
  }
  /** @brief Publish static definitions and optionally a current mathematical bearing. */
  void publish() {
    auto output = set_;
    const auto current_ns = now().nanoseconds();
    if (last_ros_now_ns_ > 0 && current_ns < last_ros_now_ns_) {
      last_pose_stamp_ns_ = 0;
    }
    last_ros_now_ns_ = current_ns;
    output.header.stamp = now();
    bool pose_valid = false;
    geometry_msgs::msg::TransformStamped pose;
    if (output.valid) {
      try {
        pose = buffer_->lookupTransform(output.header.frame_id, base_frame_, tf2::TimePointZero);
        const double age_s = (now() - rclcpp::Time(pose.header.stamp)).seconds();
        const auto & rotation = pose.transform.rotation;
        const double norm = rotation.x * rotation.x + rotation.y * rotation.y +
                            rotation.z * rotation.z + rotation.w * rotation.w;
        const auto & position = pose.transform.translation;
        const auto pose_stamp_ns = rclcpp::Time(pose.header.stamp).nanoseconds();
        if (pose_stamp_ns > last_pose_stamp_ns_ && std::isfinite(age_s) &&
          age_s >= 0 && age_s < pose_timeout_s_)
        {
          last_pose_stamp_ns_ = pose_stamp_ns;
          pose_deadline_ = std::chrono::steady_clock::now() +
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
              std::chrono::duration<double>(pose_timeout_s_ - age_s));
        }
        pose_valid = std::chrono::steady_clock::now() < pose_deadline_ && std::isfinite(age_s) && age_s >= 0 && age_s < pose_timeout_s_ &&
          std::isfinite(position.x) && std::isfinite(position.y) && std::isfinite(position.z) &&
          std::isfinite(norm) && std::abs(norm - 1.0) < 1e-3;
      } catch (const tf2::TransformException &) {
        pose_valid = false;
      }
    }
    for (auto & target : output.targets) {
      target.stop_pose.header.stamp = output.header.stamp;
      target.aim_point.header.stamp = output.header.stamp;
      if (pose_valid && target.aim_valid) {
        const auto & position = pose.transform.translation;
        const auto & rotation = pose.transform.rotation;
        const tf2::Quaternion quaternion(rotation.x, rotation.y, rotation.z, rotation.w);
        const tf2::Vector3 difference(target.aim_point.point.x - position.x,
          target.aim_point.point.y - position.y, target.aim_point.point.z - position.z);
        const auto relative = tf2::quatRotate(quaternion.inverse(), difference);
        const double bearing = std::atan2(relative.y(), relative.x());
        target.bearing_valid = std::isfinite(bearing) && std::isfinite(relative.x()) &&
          std::isfinite(relative.y()) && std::isfinite(relative.z()) && relative.x() > 0;
        if (target.bearing_valid) {
          target.bearing_rad = bearing;
        }
      }
    }
    publisher_->publish(output);
  }
  msg::TargetSet set_{};
  std::string expected_map_id_, base_frame_;
  double pose_timeout_s_{};
  int64_t last_pose_stamp_ns_{}, last_ros_now_ns_{};
  std::chrono::steady_clock::time_point pose_deadline_{};
  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::unique_ptr<tf2_ros::TransformListener> listener_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
  rclcpp::Publisher<msg::TargetSet>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace field_target_pub

/** @brief Start the typed provider; malformed approved profiles fail visibly. */
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<field_target_pub::FieldTargetNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("field_target_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
