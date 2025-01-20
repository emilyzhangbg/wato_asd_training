#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <cmath>
#include <vector>
#include <algorithm>

namespace robot
{

class MapMemoryCore {
  public:
    explicit MapMemoryCore(const rclcpp::Logger& logger);

    // Callback for costmap and Odometry
    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    // Map update logic
   void updateMap(const rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr& map_pub);

  private:
    rclcpp::Logger logger_;
    // Global map and tracking variables
    nav_msgs::msg::OccupancyGrid global_map;
    nav_msgs::msg::OccupancyGrid latest_costmap;
    //Robot tracking variables
    double last_x = 0.0;
    double last_y = 0.0;

     // Minimum distance to trigger map updates
    const double distance_threshold = 0.5; 

    bool costmap_updated = false;
    bool should_update_map = false;

    // Helper methods
    void integrateCostmap();
};

}  

#endif  
