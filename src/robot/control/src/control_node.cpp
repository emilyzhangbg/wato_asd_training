#include "control_node.hpp"

using std::placeholders::_1;

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore(this->get_logger())) {
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>("/path", 10,
                          std::bind(&ControlNode::recvPath, this, _1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", 10,
                          std::bind(&ControlNode::recvOdom, this, _1));
  twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                          std::bind(&ControlNode::timerCallback, this));
}

void ControlNode::recvPath(nav_msgs::msg::Path::SharedPtr msg) {
  control_.setPath(msg);
}

void ControlNode::recvOdom(nav_msgs::msg::Odometry::SharedPtr msg) {
  control_.setOdom(msg);
}

void ControlNode::timerCallback(){
  control_.move(twist_pub_);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}

