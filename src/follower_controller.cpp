#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <cmath>

class FollowerController : public rclcpp::Node {
public:
  FollowerController() : Node("follower_controller") {
    buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);

    pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle2/cmd_vel",
                                                             10);
    timer_ =
        this->create_wall_timer(std::chrono::milliseconds(100),
                                std::bind(&FollowerController::loop, this));

    RCLCPP_INFO(this->get_logger(), "Follower controller started");
    // Gains - tune these if needed
    k_lin_ = 1.2;
    k_ang_ = 4.0;
    max_lin_ = 2.0;
    max_ang_ = 6.0;
  }

private:
  void loop() {
    try {
      // get transform from turtle2 frame to leader (coordinates of leader in
      // turtle2 frame)
      geometry_msgs::msg::TransformStamped t =
          buffer_->lookupTransform("turtle2", "leader", tf2::TimePointZero);

      double dx = t.transform.translation.x;
      double dy = t.transform.translation.y;

      double dist = std::hypot(dx, dy);
      double angle = std::atan2(dy, dx);

      geometry_msgs::msg::Twist cmd;
      double lin = k_lin_ * dist;
      double ang = k_ang_ * angle;

      if (lin > max_lin_)
        lin = max_lin_;
      if (ang > max_ang_)
        ang = max_ang_;
      if (lin < -max_lin_)
        lin = -max_lin_;
      if (ang < -max_ang_)
        ang = -max_ang_;

      cmd.linear.x = lin;
      cmd.angular.z = ang;

      pub_->publish(cmd);
    } catch (const std::exception &e) {
      // transform not available yet
      // RCLCPP_DEBUG(this->get_logger(), "TF lookup failed: %s", e.what());
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  double k_lin_, k_ang_, max_lin_, max_ang_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FollowerController>());
  rclcpp::shutdown();
  return 0;
}
