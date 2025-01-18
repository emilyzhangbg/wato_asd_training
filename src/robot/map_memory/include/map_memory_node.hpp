#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/Path.hpp"
#include "map_memory_core.hpp"

class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();

  private:
    robot::MapMemoryCore map_memory_;

    // Constructs for ROS
    rclcpp::Subscription<nav_msgs::OccupancyGrid>:: SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::Odomentry>:: SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif 
