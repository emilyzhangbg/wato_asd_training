#include "control_core.hpp"

#include <utility>
#include <limits.h>

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger& logger) : logger_(logger) {
    lookahead_dist_ = 1.1;
    goal_tolerance_ = 0.5;
    linear_speed_ = 0.3;
    wheel_base_ = 1.0;
}

void ControlCore::controlLoop(rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_) {
    // Skip control if no path data is available
    if (path_.poses.empty()) {
        return;
    }

    // Skip control if already at goal
    if (computeDistance(path_.poses[path_.poses.size() - 1].pose.position, robot_pose_.position) < goal_tolerance_) {
        RCLCPP_INFO(logger_, "Arrived at goal!");
        // Stop robot
        geometry_msgs::msg::Twist stop;
        stop.linear.x = 0;
        stop.angular.z = 0;
        twist_pub_->publish(stop);

        return;
    }

    // Find the lookahead point
    auto lookahead_point = findLookaheadPoint();
    if (!lookahead_point) {
        RCLCPP_WARN(logger_, "Cannot find look ahead point!");
        return;  // No valid lookahead point found
    }

    // Compute velocity command
    auto cmd_vel = computeVelocity(*lookahead_point);

    // Publish the velocity command
    twist_pub_->publish(cmd_vel);
}

std::optional<geometry_msgs::msg::PoseStamped> ControlCore::findLookaheadPoint() {
    double min_distance = INT_MAX;
    int min_idx = -1;
    for (int i = path_.poses.size() - 1; i >= 0; i--) {
        double distance = computeDistance(path_.poses[i].pose.position, robot_pose_.position);

        if (distance >= lookahead_dist_ && distance < min_distance) {
            min_distance = distance;
            min_idx = i;
        }
    }

    return min_idx == -1 ? std::nullopt : std::optional<geometry_msgs::msg::PoseStamped>{path_.poses[min_idx]};
}

geometry_msgs::msg::Twist ControlCore::computeVelocity(const geometry_msgs::msg::PoseStamped &target) {
    geometry_msgs::msg::Twist cmd_vel;

    double target_x = target.pose.position.x;
    double target_y = target.pose.position.y;
    double cur_x = robot_pose_.position.x;
    double cur_y = robot_pose_.position.y;

    double dx = target_x - cur_x;
    double dy = target_y - cur_y;
    double angle = std::atan2(dy, dx) - extractYaw(robot_pose_.orientation);

    if (angle > M_PI_2)
        angle -= M_PI;
    else if (angle < -1 * M_PI_2)
        angle += M_PI;

    double distance = computeDistance(target.pose.position, robot_pose_.position);
    double steering_angle = std::atan((2.0 * wheel_base_ * std::sin(angle)) / distance);

    if (steering_angle > 0.6) {
        steering_angle = 0.6;
    }
    else if (steering_angle < -0.6) {
        steering_angle = -0.6;
    }

    cmd_vel.linear.x = linear_speed_;
    cmd_vel.angular.z = steering_angle;

    return cmd_vel;
}

double ControlCore::computeDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b) {
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}

double ControlCore::extractYaw(const geometry_msgs::msg::Quaternion &quat) {
    double x = quat.x;
    double y = quat.y;
    double z = quat.z;
    double w = quat.w;
    
    return std::atan2(2.0 * (w*z + x*y), 1.0 - 2.0 * (x*x + y*y));
}

void ControlCore::setPath(nav_msgs::msg::Path::SharedPtr msg) {
    path_ = *msg;
}

void ControlCore::setOdom(nav_msgs::msg::Odometry::SharedPtr msg) {
    robot_pose_ = msg->pose.pose;
}

}
