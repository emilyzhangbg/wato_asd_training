#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"

namespace robot
{

class ControlCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    ControlCore(const rclcpp::Logger& logger);

    void move(rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_);
    void setPath(nav_msgs::msg::Path::SharedPtr msg);
    void setOdom(nav_msgs::msg::Odometry::SharedPtr msg);

  private:
    rclcpp::Logger logger_;

    nav_msgs::msg::Path::SharedPtr path_;
    nav_msgs::msg::Odometry::SharedPtr odom_;
    
    double lookahead_dist_;
    double goal_tolerance_;
    double linear_speed_;
};

} 

#endif 
