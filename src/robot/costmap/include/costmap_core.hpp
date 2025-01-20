#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "std_msgs/msg/string.hpp"

namespace robot
{

class CostmapCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger& logger, rclcpp::Clock::SharedPtr clock);
    void publishCostmap(const sensor_msgs::msg::LaserScan::SharedPtr laserscan);
    void inflate(std::vector<int8_t>& grid, int res, double radius);
    void set(std::vector<int8_t>& grid, int res, int y, int x, int8_t val);
    int8_t get(std::vector<int8_t> grid, int res, int y, int x);
    void setPose(const nav_msgs::msg::Odometry::SharedPtr odom);

  private:
    rclcpp::Logger logger_;
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
    double robot_angle;
    double robot_angle_w;
    double robot_x;
    double robot_y;
    double robot_z;
};

}  

#endif  