#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

class FieldTargetNode : public rclcpp::Node
{
public:
    FieldTargetNode() : Node("field_target_node")
    {
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&FieldTargetNode::init_and_publish, this)
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
        // タイマーが何回も動かないように即座にキャンセルする
        timer_->cancel();

        // ノードが完全に立ち上がった安全な状態でブロードキャスターを初期化
        tf_static_broadcaster_ =
            std::make_shared<tf2_ros::StaticTransformBroadcaster>(shared_from_this());

        publish_static_transforms();
    }

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
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
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FieldTargetNode>());
    rclcpp::shutdown();
    return 0;
}