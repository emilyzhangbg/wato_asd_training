#include "map_memory_core.hpp"

using namespace std;
namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger) 
  : logger_(logger) {}

// Costmap callback function
void MapMemoryCore::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  // Printing the text on console
  RCLCPP_INFO(this->logger_, "Received Costmap");
// Store the latest costmap (From psuedocode)
  latest_costmap = *msg;
  costmap_updated = true;
}

// Callback for odometry updates from psuedocode
void MapMemoryCore::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;

    // Compute distance traveled
    double distance = sqrt(pow(x - last_x, 2) + pow(y - last_y, 2));
    if (distance >= distance_threshold) {
        last_x = x;
        last_y = y;
        should_update_map = true;
    }
}

// Timer-based map update
void MapMemoryCore::updateMap(const rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr& map_pub) {
    if (should_update_map && costmap_updated) {
        integrateCostmap();
        map_pub->publish(global_map);
        should_update_map = false;
        costmap_updated = false;
    }
}

void MapMemoryCore::integrateCostmap() {
  // Initial Integration of costmap into global map
  if (global_map.data.empty()) {
     // Set map metadata
        global_map.info.resolution = latest_costmap.info.resolution;
        global_map.info.width = 200;  
        global_map.info.height = 200; 
        // Bottom-left corner in global frame
        global_map.info.origin.position.x = -10.0; 
        global_map.info.origin.position.y = -10.0;
        // Currently no rotation
        global_map.info.origin.orientation.w = 1.0; 

        // Initialize all cells to unknown (-1)
        global_map.data.resize(global_map.info.width * global_map.info.height, -1);

        return;  // Exit early since no costmap merging is needed yet
  }
  for (int i = 0; i < global_map.info.height; ++i) {
    for (int j = 0; j < global_map.info.width; ++j) {
      int costmap_index = j * latest_costmap.info.width + i;
      // Transformation of local coordinates to global coordinates
      double local_x = latest_costmap.info.origin.position.x + i * latest_costmap.info.resolution;
      double local_y = latest_costmap.info.origin.position.y + j * latest_costmap.info.resolution;

      if (global_x < 0 || global_y < 0 || global_x >= global_map.info.width || global_y >= global_map.info.height) {
        continue;  // Skip out-of-bounds cells
      }

      int global_index = global_y * global_map.info.width + global_x;

      // Merge data: overwrite only if the costmap cell has known data
      if (latest_costmap.data[costmap_index] != -1) {
        global_map.data[global_index] = latest_costmap.data[costmap_index];
      }
    }
  }
  

}


}