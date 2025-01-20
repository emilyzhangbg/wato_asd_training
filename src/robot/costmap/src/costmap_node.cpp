#include <chrono>
#include <memory>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "costmap_node.hpp"

using std::placeholders::_1;

static const double resolution     = 0.1;    // meters per cell
static const int    map_width      = 300;    // covers 30 m in X
static const int    map_height     = 300;    // covers 30 m in Y
static const double map_origin_x   = -15.0;  // bottom-left corner in sim_world
static const double map_origin_y   = -15.0;  // covers [-15..+15]
static const double inflation_cells= 2.1;    // interpret as ~2 cells if you keep the same inflate logic
// If you want 2.1 meters of inflation, call inflate(grid, map_width, 21) or similar.

CostmapNode::CostmapNode() 
: Node("costmap"), 
  costmap_(robot::CostmapCore(this->get_logger())) 
{
  // Publishers/Subscribers
  string_pub_ = this->create_publisher<std_msgs::msg::String>("/test_topic", 10);

  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 
    10, 
    std::bind(&CostmapNode::publishCostmap, this, _1)
  );

  odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 
    10, 
    std::bind(&CostmapNode::setPose, this, _1)
  );

  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
  past_ = this->get_clock()->now();
}

void CostmapNode::publishCostmap(const sensor_msgs::msg::LaserScan::SharedPtr scan) 
{
  // Create a grid of fixed size map_width x map_height
  // representing a 30x30 environment in "sim_world"
  std::vector<int8_t> grid(map_width * map_height, 0);

  // For each laser reading, transform from robot frame to global sim_world
  for (size_t i = 0; i < scan->ranges.size(); i++) {
    double range = scan->ranges[i];
    if (range < scan->range_min || range > scan->range_max) {
      continue;  // skip invalid ranges
    }

    // Laser angle in the robot's local frame
    double angle_local = scan->angle_min + i * scan->angle_increment;

    // Convert LaserScan to local coordinates
    double x_local = range * std::cos(angle_local);
    double y_local = range * std::sin(angle_local);

    // Transform local -> global (sim_world) using the robot pose
    // robot_angle, robot_x, robot_y are updated in setPose()
    double x_world = robot_x + ( x_local * std::cos(robot_angle)
                               - y_local * std::sin(robot_angle));
    double y_world = robot_y + ( x_local * std::sin(robot_angle)
                               + y_local * std::cos(robot_angle));

    // Convert global coords -> grid cells
    int gx = static_cast<int>((x_world - map_origin_x) / resolution);
    int gy = static_cast<int>((y_world - map_origin_y) / resolution);

    // Bound-check
    if (gx >= 0 && gx < map_width && gy >= 0 && gy < map_height) {
      grid[gy * map_width + gx] = 100;  // Mark as obstacle
    }
  }

  // Inflate obstacles (currently we treat 'inflation_cells' as # of cells)
  inflate(grid, map_width, inflation_cells);

  // Build the OccupancyGrid message
  nav_msgs::msg::OccupancyGrid occgrid;
  occgrid.header.stamp = this->get_clock()->now();
  occgrid.header.frame_id = "sim_world";  // global, non-moving frame

  occgrid.info.map_load_time = this->get_clock()->now();
  occgrid.info.resolution = resolution; // 0.1 m/cell
  occgrid.info.width = map_width;
  occgrid.info.height = map_height;

  // The map covers [-15..+15] in X and Y
  occgrid.info.origin.position.x = map_origin_x;
  occgrid.info.origin.position.y = map_origin_y;
  occgrid.info.origin.orientation.w = 1.0;

  // Assign the data
  occgrid.data = grid;

  // Publish the costmap
  costmap_pub_->publish(occgrid);
}

void CostmapNode::inflate(std::vector<int8_t>& grid, int res, double radius) 
{
  int intrad = static_cast<int>(std::ceil(radius));

  for (int y = 0; y < res; y++) {
    for (int x = 0; x < res; x++) {
      if (grid[y * res + x] == 100) {
        // For each cell within the inflation radius
        for (int dy = -intrad; dy <= intrad; dy++) {
          for (int dx = -intrad; dx <= intrad; dx++) {
            double dist = std::sqrt(static_cast<double>(dy*dy + dx*dx));
            if (dist < radius) {
              int8_t curcost = get(grid, res, y + dy, x + dx);
              // cost formula: cost = 100 * (1 - dist/radius)
              double newcost_d = 100.0 * (1.0 - (dist / radius));
              if (newcost_d < 0.0) newcost_d = 0.0;
              if (newcost_d > 100.0) newcost_d = 100.0;
              int8_t newcost = static_cast<int8_t>(std::round(newcost_d));

              if (newcost > curcost) {
                set(grid, res, y + dy, x + dx, newcost);
              }
            }
          }
        }
      }
    }
  }
}

void CostmapNode::set(std::vector<int8_t>& grid, int res, int y, int x, int8_t val) 
{
  // Bounds check
  if (y >= 0 && y < res && x >= 0 && x < res) {
    grid[y * res + x] = val;
  }
}

int8_t CostmapNode::get(const std::vector<int8_t>& grid, int res, int y, int x) 
{
  if (y >= 0 && y < res && x >= 0 && x < res) {
    return grid[y * res + x];
  }
  return 100; // default obstacle if out-of-bounds
}

void CostmapNode::setPose(const nav_msgs::msg::Odometry::SharedPtr odom) 
{
  tf2::Quaternion quat_tf;
  tf2::fromMsg(odom->pose.pose.orientation, quat_tf);
  double roll{}, pitch{}, yaw{};
  tf2::Matrix3x3 m(quat_tf);
  m.getRPY(roll, pitch, yaw);

  // Save the robot's global pose (sim_world coords)
  robot_angle = yaw;
  robot_x = odom->pose.pose.position.x;
  robot_y = odom->pose.pose.position.y;
  robot_z = odom->pose.pose.position.z;
}

// Optional string-based debug printing
void CostmapNode::print(std::string s) 
{
  auto message = std_msgs::msg::String();
  message.data = s;
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  string_pub_->publish(message);
}

// Standard main for this node
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
