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

    void controlLoop(rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_);
    void setPath(nav_msgs::msg::Path::SharedPtr msg);
    void setOdom(nav_msgs::msg::Odometry::SharedPtr msg);

private:
    std::optional<geometry_msgs::msg::PoseStamped> findLookaheadPoint();
    geometry_msgs::msg::Twist computeVelocity(const geometry_msgs::msg::PoseStamped &target);
    double computeDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b);
    double extractYaw(const geometry_msgs::msg::Quaternion &quat);

    rclcpp::Logger logger_;

    nav_msgs::msg::Path path_;
    geometry_msgs::msg::Pose robot_pose_;

    double lookahead_dist_;
    double goal_tolerance_;
    double linear_speed_;
    double wheel_base_;
};

} 

#endif 
