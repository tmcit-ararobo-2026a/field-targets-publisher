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

        // 起動直後に1回だけ静的TFを配信するタイマー
        init_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&FieldTargetNode::init_and_publish, this)
        );

        // 定期的に（例: 1秒ごとに）相対位置を調べるタイマー
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
    void init_and_publish()
    {
        init_timer_->cancel();

        tf_static_broadcaster_ =
            std::make_shared<tf2_ros::StaticTransformBroadcaster>(shared_from_this());

        publish_static_transforms();
    }

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::TimerBase::SharedPtr init_timer_;
    rclcpp::TimerBase::SharedPtr lookup_timer_;
    rclcpp::TimerBase::SharedPtr field_timer_;

    bool fielda;
    void publish_static_transforms()
    {
        // 提示されたすべてのオブジェクトデータのリスト
        std::vector<TargetInfo> targets = {
            // 旗
            {    "flag_a_base",   0.55,  3.025,   (0.0 + 0.18) / 2.0},
            {    "flag_b_base",   0.55, -3.025,   (0.0 + 0.18) / 2.0},

            // 机
            {       "desk_1_a", -2.295,  3.855,   (0.0 + 0.76) / 2.0},
            {       "desk_1_b", -2.295, -3.855,   (0.0 + 0.76) / 2.0},
            {       "desk_2_a",  3.395,  3.855,   (0.0 + 0.76) / 2.0},
            {       "desk_2_b",  3.395, -3.855,   (0.0 + 0.76) / 2.0},
            {       "desk_3_a", -4.895,  5.445,   (0.0 + 0.76) / 2.0},
            {       "desk_3_b", -4.895, -5.445,   (0.0 + 0.76) / 2.0},
            {       "desk_4_a", -4.750,  1.105,   (0.0 + 0.76) / 2.0},
            {       "desk_4_b", -4.750, -1.105,   (0.0 + 0.76) / 2.0},

            // 固定バケツ①
            {     "bucket_1_a",   0.55,   0.87,  (0.0 + 0.255) / 2.0},
            {     "bucket_1_b",   0.55,  -0.87,  (0.0 + 0.255) / 2.0},

            // 固定バケツ②
            {"bucket_2_a_base",  -1.27,   1.48,   (0.0 + 0.60) / 2.0},
            {"bucket_2_a_body",  -1.27,   1.48, (0.60 + 0.855) / 2.0},
            {"bucket_2_b_base",  -1.27,  -1.48,   (0.0 + 0.60) / 2.0},
            {"bucket_2_b_body",  -1.27,  -1.48, (0.60 + 0.855) / 2.0},

            // 固定バケツ③
            {"bucket_3_a_base",   2.37,   1.48,   (0.0 + 0.30) / 2.0},
            {"bucket_3_a_body",   2.37,   1.48, (0.30 + 0.555) / 2.0},
            {"bucket_3_b_base",   2.37,  -1.48,   (0.0 + 0.30) / 2.0},
            {"bucket_3_b_body",   2.37,  -1.48, (0.30 + 0.555) / 2.0}
        };
        for (const auto& target : targets) {
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
        RCLCPP_INFO(this->get_logger(), "tf_finish");
    }

    void lookup_target_position()
    {
        try {
            geometry_msgs::msg::TransformStamped target_transform =
                tf_buffer_->lookupTransform("base_link", "bucket_1_a", tf2::TimePointZero);
            double x = target_transform.transform.translation.x;
            double y = target_transform.transform.translation.y;
            RCLCPP_INFO(this->get_logger(), "%f,%f", x, y);
        } catch (tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
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