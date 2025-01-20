#include <chrono>
#include <memory>
#include <string>
#include <algorithm>  // for std::fill

#include "costmap_node.hpp"

const int GRIDSIZE = 300;

// Constructor
CostmapNode::CostmapNode() : Node("costmap")
{
  string_pub_ = this->create_publisher<std_msgs::msg::String>("/test_topic", 50);
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10, std::bind(&CostmapNode::lidar_sub, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&CostmapNode::odom_sub, this, std::placeholders::_1));
  grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
}

// 1) Inflation with partial cost range [50..100]
void CostmapNode::inflateObstacles(int grid[300][300], double inflationRadius, int maxCost)
{
  // inflationRadius is in meters? Then each cell is 0.1m => 1 meter = 10 cells
  // or if your code treats it differently, keep that logic.
  int inflationCells = static_cast<int>(std::round(inflationRadius * 10));

  for (int x = 0; x < GRIDSIZE; x++) {
    for (int y = 0; y < GRIDSIZE; y++) {
      if (grid[x][y] == 100) {
        // For each cell in the inflation square
        for (int dx = -inflationCells; dx <= inflationCells; dx++) {
          for (int dy = -inflationCells; dy <= inflationCells; dy++) {
            int nx = x + dx;
            int ny = y + dy;
            // Check bounds
            if (nx >= 0 && nx < GRIDSIZE && ny >= 0 && ny < GRIDSIZE) {
              double distance = std::sqrt(dx * dx + dy * dy) / 10.0; // each cell=0.1m

              if (distance > inflationRadius) {
                continue;  // beyond inflation range
              }
              // cost in range [50..100], near obstacle => 100, outer => ~50
              double partial_cost = 50.0 + 50.0 * (1.0 - (distance / inflationRadius));
              if (partial_cost < 50.0) partial_cost = 50.0;
              if (partial_cost > 100.0) partial_cost = 100.0;

              int cost = static_cast<int>(std::round(partial_cost));
              if (cost > grid[nx][ny]) {
                grid[nx][ny] = cost;
              }
            }
          }
        }
      }
    }
  }
}

void CostmapNode::odom_sub(const nav_msgs::msg::Odometry::SharedPtr odom)
{
  float float_x = odom->pose.pose.position.x;
  float float_y = odom->pose.pose.position.y;

  // Convert orientation to yaw
  double x = odom->pose.pose.orientation.x;
  double y = odom->pose.pose.orientation.y;
  double z = odom->pose.pose.orientation.z;
  double w = odom->pose.pose.orientation.w;

  double yaw = std::atan2(2.0*(w*z + x*y), 1.0 - 2.0*(y*y+z*z));

  // We store robot coords in "tenths of meter" + offset => see below in lidar_sub
  this->x_ = float_x * 10;
  this->y_ = float_y * 10;
  this->dir_x_ = std::cos(yaw);
  this->dir_y_ = std::sin(yaw);
  this->dir_ = std::atan2(this->dir_y_, this->dir_x_);
}

void CostmapNode::lidar_sub(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  // If we haven't set a valid pose yet, skip
  if (this->x_ < 0 && this->dir_x_ < -99) {
    return;
  }

  // The array
  // 2) Mark unobserved as -1
  int array[GRIDSIZE][GRIDSIZE];
  std::fill(&array[0][0], &array[0][0] + GRIDSIZE*GRIDSIZE, -1);

  // Robot yaw
  float angle = std::atan2(this->dir_y_, this->dir_x_);

  // Step 2: Convert LaserScan => obstacles
  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    double laser_angle = angle + scan->angle_min + i * scan->angle_increment;
    double range = scan->ranges[i];
    if (range < scan->range_max && range > scan->range_min) {
      // Convert to "grid" coords
      int x_grid = static_cast<int>(range*std::cos(laser_angle)*10 + this->x_) + 150;
      int y_grid = static_cast<int>(range*std::sin(laser_angle)*10 + this->y_) + 150;

      if (x_grid < GRIDSIZE && x_grid >= 0 && y_grid < GRIDSIZE && y_grid >= 0) {
        array[x_grid][y_grid] = 100;  // obstacle
      }
    }
  }

  // Step 3: Inflate
  inflateObstacles(array, 1.6, 100);

  // Step 4: Publish costmap
  nav_msgs::msg::OccupancyGrid msg;
  // 4a) Use "sim_world" for consistency with your environment
  msg.header.frame_id = "sim_world";  
  // We can reuse LaserScan's timestamp or use now()
  // msg.header.stamp = scan->header.stamp;
  msg.header.stamp = this->now();

  msg.info.width  = GRIDSIZE;
  msg.info.height = GRIDSIZE;
  msg.info.resolution = 0.1;
  // Place bottom-left corner at (-15,-15)
  msg.info.origin.position.x = -15.0;
  msg.info.origin.position.y = -15.0;
  msg.info.origin.orientation.w = 1.0;

  msg.data.resize(GRIDSIZE * GRIDSIZE);
  // Flatten
  for (int x = 0; x < GRIDSIZE; ++x) {
    for (int y = 0; y < GRIDSIZE; ++y) {
      msg.data[y*GRIDSIZE + x] = array[x][y];
    }
  }

  grid_pub_->publish(msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
