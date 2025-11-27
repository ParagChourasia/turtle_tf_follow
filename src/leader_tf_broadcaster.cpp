#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "turtlesim/msg/pose.hpp"
#include <tf2/LinearMath/Quaternion.h>

class LeaderTFBroadcaster : public rclcpp::Node {
public:
  LeaderTFBroadcaster() : Node("leader_tf_broadcaster") {
    br_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    sub_ = this->create_subscription<turtlesim::msg::Pose>(
        "/turtle1/pose", 10,
        std::bind(&LeaderTFBroadcaster::pose_callback, this,
                  std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(),
                "Leader TF broadcaster started (subscribing to /turtle1/pose)");
  }

private:
  void pose_callback(const turtlesim::msg::Pose::SharedPtr msg) {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "world";
    t.child_frame_id = "leader";

    t.transform.translation.x = msg->x;
    t.transform.translation.y = msg->y;
    t.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, msg->theta);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    br_->sendTransform(t);
  }

  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> br_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LeaderTFBroadcaster>());
  rclcpp::shutdown();
  return 0;
}
