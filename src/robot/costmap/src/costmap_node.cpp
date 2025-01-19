#include <chrono>
#include <memory>
 
#include "costmap_node.hpp"

using std::placeholders::_1;
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger(), this->get_clock())) {
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/lidar", 10, std::bind(&CostmapNode::recvLidar, this, _1));
  odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", 10, std::bind(&CostmapNode::recvOdom, this, _1));
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
}

void CostmapNode::recvLidar(sensor_msgs::msg::LaserScan::SharedPtr msg) {
  costmap_.publishCostmap(msg);
}

void CostmapNode::recvOdom(nav_msgs::msg::Odometry::SharedPtr msg) {
  costmap_.setPose(msg);
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}