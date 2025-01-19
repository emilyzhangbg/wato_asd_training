#include "control_core.hpp"

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger& logger) : logger_(logger) {
  lookahead_dist_ = 0.5;
  goal_tolerance_ = 0.1;
  linear_speed_ = 0.4;
}


void ControlCore::move(rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_) {
  auto delta = geometry_msgs::msg::Twist();
  delta.linear.x = linear_speed_;

  
}

}
