#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

class FieldTargetNode : public rclcpp::Node
{
public:
    FieldTargetNode() : Node("field_target_node")
    {
        tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        targets_     = {
            // 領域A
            {    "flag_a_base",   0.55,  3.025,   (0.0 + 0.18) / 2.0},
            {       "desk_1_a", -2.295,  3.855,   (0.0 + 0.76) / 2.0},
            {       "desk_2_a",  3.395,  3.855,   (0.0 + 0.76) / 2.0},
            {       "desk_3_a", -4.895,  5.445,   (0.0 + 0.76) / 2.0},
            {       "desk_4_a", -4.750,  1.105,   (0.0 + 0.76) / 2.0},
            {     "bucket_1_a",   0.55,   0.87,  (0.0 + 0.255) / 2.0},
            {"bucket_2_a_base",  -1.27,   1.48,   (0.0 + 0.60) / 2.0},
            {"bucket_2_a_body",  -1.27,   1.48, (0.60 + 0.855) / 2.0},
            {"bucket_3_a_base",   2.37,   1.48,   (0.0 + 0.30) / 2.0},
            {"bucket_3_a_body",   2.37,   1.48, (0.30 + 0.555) / 2.0},

            // 領域B
            {    "flag_b_base",   0.55, -3.025,   (0.0 + 0.18) / 2.0},
            {       "desk_1_b", -2.295, -3.855,   (0.0 + 0.76) / 2.0},
            {       "desk_2_b",  3.395, -3.855,   (0.0 + 0.76) / 2.0},
            {       "desk_3_b", -4.895, -5.445,   (0.0 + 0.76) / 2.0},
            {       "desk_4_b", -4.750, -1.105,   (0.0 + 0.76) / 2.0},
            {     "bucket_1_b",   0.55,  -0.87,  (0.0 + 0.255) / 2.0},
            {"bucket_2_b_base",  -1.27,  -1.48,   (0.0 + 0.60) / 2.0},
            {"bucket_2_b_body",  -1.27,  -1.48, (0.60 + 0.855) / 2.0},
            {"bucket_3_b_base",   2.37,  -1.48,   (0.0 + 0.30) / 2.0},
            {"bucket_3_b_body",   2.37,  -1.48, (0.30 + 0.555) / 2.0}
        };

        init_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&FieldTargetNode::init_and_publish, this)
        );

        lookup_timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&FieldTargetNode::lookup_target_position, this)
        );
        field_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&FieldTargetNode::field_search, this)
        );
    }

private:
    struct TargetInfo {
        std::string name;
        double x;
        double y;
        double z;
    };
    struct TargetData {
        std::string name;
        double angle_rad;
    };
    std::vector<TargetInfo> targets_;

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::TimerBase::SharedPtr init_timer_;
    rclcpp::TimerBase::SharedPtr lookup_timer_;
    rclcpp::TimerBase::SharedPtr field_timer_;

    bool fielda = true;

    void init_and_publish()
    {
        init_timer_->cancel();

        tf_static_broadcaster_ =
            std::make_shared<tf2_ros::StaticTransformBroadcaster>(shared_from_this());

        publish_static_transforms();
    }

    void publish_static_transforms()
    {
        for (const auto& target : targets_) {
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp    = this->get_clock()->now();
            t.header.frame_id = "map";
            t.child_frame_id  = target.name;

            t.transform.translation.x = target.x;
            t.transform.translation.y = target.y;
            t.transform.translation.z = target.z;

            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, 0.0);
            t.transform.rotation.x = q.x();
            t.transform.rotation.y = q.y();
            t.transform.rotation.z = q.z();
            t.transform.rotation.w = q.w();

            tf_static_broadcaster_->sendTransform(t);
        }
        RCLCPP_INFO(this->get_logger(), "tf_map_finish");
    }

    void lookup_target_position()
    {
        std::string target_suffix  = fielda ? "_b" : "_a";
        const double threshold_rad = 20.0 * M_PI / 180.0;
        std::vector<TargetData> found_targets;

        for (const auto& target : targets_) {
            if (target.name.rfind(target_suffix) == std::string::npos) {
                continue;  // 自陣側のオブジェクトならスキップ
            }

            try {
                geometry_msgs::msg::TransformStamped targets_position =
                    tf_buffer_->lookupTransform("base_link", target.name, tf2::TimePointZero);
                double x = targets_position.transform.translation.x;
                double y = targets_position.transform.translation.y;

                double angle_rad = std::atan2(y, x);

                if (x >= 0 && angle_rad >= -threshold_rad && angle_rad <= threshold_rad) {
                    found_targets.push_back({target.name, angle_rad});
                }
            } catch (tf2::TransformException& ex) {
            }
        }

        try {
            geometry_msgs::msg::TransformStamped opp_transform =
                tf_buffer_->lookupTransform("base_link", "opponent_robot", tf2::TimePointZero);

            double x = opp_transform.transform.translation.x;
            double y = opp_transform.transform.translation.y;

            double angle_rad = std::atan2(y, x);
            if (x >= 0 && angle_rad >= -threshold_rad && angle_rad <= threshold_rad) {
                found_targets.push_back({"opponent_robot", angle_rad});
            }

        } catch (tf2::TransformException& ex) {
        }
        if (found_targets.empty()) {
            RCLCPP_INFO(this->get_logger(), "99rad");
        }

        if (found_targets.size() > 4) {
            found_targets.resize(4);
        }

        // 4. 最大4つになった角度（rad）をシンプルに表示
        for (const auto& t : found_targets) {
            RCLCPP_INFO(this->get_logger(), "%.2f", t.angle_rad);
        }
    }
    void field_search()
    {
        try {
            geometry_msgs::msg::TransformStamped field_transform =
                tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
            if (field_transform.transform.translation.y >= 0 &&
                field_transform.transform.translation.y <= 5.7) {
                fielda = true;
            } else if (
                field_transform.transform.translation.y <= 0 &&
                field_transform.transform.translation.y >= -5.7
            ) {
                fielda = false;
            }
            RCLCPP_INFO(this->get_logger(), "%d", fielda);
            field_timer_->cancel();
        } catch (tf2::TransformException& ex) {
            RCLCPP_WARN(
                this->get_logger(), "Waiting for robot position to determine area: %s", ex.what()
            );
        }
    }
};
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FieldTargetNode>());
    rclcpp::shutdown();
    return 0;
}
