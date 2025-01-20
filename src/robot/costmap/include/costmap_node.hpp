#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "costmap_core.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class CostmapNode : public rclcpp::Node {
public:
  CostmapNode();

  void lidar_sub(const sensor_msgs::msg::LaserScan::SharedPtr scan);
  void odom_sub(const nav_msgs::msg::Odometry::SharedPtr odom);

  // If you have other functions, keep them here...
  // e.g. publishMessage, etc.

private:
  // Helper function to inflate obstacles
  // (Now returns partial costs in [50..100])
  void inflateObstacles(int grid[300][300], double inflationRadius, int maxCost);

  // For storing robot pose
  double x_ = -1.0;
  double y_ = -1.0;
  double dir_x_ = -100.0;
  double dir_y_ = -100.0;
  double dir_ = 0.0;

  // ROS publishers/subscribers
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
};

#endif  // COSTMAP_NODE_HPP_
