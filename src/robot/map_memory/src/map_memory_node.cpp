#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_((this->get_logger())) {
  // subscriber initialization
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/costmap", 10, 
    std::bind(&robot::MapMemoryCore::costmapCallback, &map_memory_, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", 10, 
    std::bind(&robot::MapMemoryCore::odomCallback, &map_memory_, std::placeholders::_1));
  
  // publisher initialization
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

  // Initialize timer
  timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&MapMemoryNode::timerCallback, this));
}

void MapMemoryNode::timerCallback()
{
  // We simply call the core's updateMap function,
  // passing the publisher we created.
  map_memory_.updateMap(map_pub_);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
