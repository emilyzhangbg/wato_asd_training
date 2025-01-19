#include "costmap_core.hpp"

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger) : logger_(logger) {}

void CostmapCore::publishCostmap(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  /*
    This formula below ensures that at the maximum distance,
    the maximum gap between laser scans is exactly one cell
  */
  int res = 2 / scan->angle_increment;

  /* Scan */
  std::vector<int8_t> grid(res * res, 0);
  for (int i = 0; i < scan->ranges.size(); i++) {
    double angle = scan->angle_min + i * scan->angle_increment + robot_angle;
    double range = scan->ranges[i];
    if (range > scan->range_min && range < scan->range_max) {
      int grid_x = res / 2 + (range / scan->range_max) * -sin(angle) * (res / 2);
      int grid_y = res / 2 + (range / scan->range_max) * -cos(angle) * (res / 2);

      grid[res * grid_y + grid_x] = 100;
    }
  }

  inflate(grid, res, 2.1);

  /* Set up OccupancyGrid */
  auto occgrid = nav_msgs::msg::OccupancyGrid();
  occgrid.data = grid;
  occgrid.info.resolution = 2 * scan->range_max / res;
  occgrid.info.width = res;
  occgrid.info.height = res;
  occgrid.info.origin.position.x = robot_x - scan->range_max;
  occgrid.info.origin.position.y = robot_y - scan->range_max;
  occgrid.info.origin.position.z = robot_z;
  occgrid.info.origin.orientation.x = 0;
  occgrid.info.origin.orientation.y = 0;
  occgrid.info.origin.orientation.z = 0;
  occgrid.info.origin.orientation.w = 1.0;
  
  costmap_pub_->publish(occgrid);
}

void CostmapCore::inflate(std::vector<int8_t>& grid, int res, double radius) {
  int intrad = ceil(radius);
  for (int y = 0; y < res; y++) {
    for (int x = 0; x < res; x++) {
      if (grid[y * res + x] == 100) {
        for (int i = -intrad; i <= intrad; i++) {
          for (int j = -intrad; j <= intrad; j++) {
            double dist = sqrt(i * i + j * j);

            if (dist < radius) {
              int curcost = get(grid, res, y + i, x + j);
              int newcost = 100.0 * (1.0 - dist / radius);

              if (newcost > curcost) {
                set(grid, res, y + i, x + j, newcost);
              }
            }
          }
        }
      }
    }
  }
}
void CostmapCore::set(std::vector<int8_t>& grid, int res, int y, int x, int8_t val) {
  if (y >= 0 && y < res && x >= 0 && x < res) {
    grid[res * y + x] = val;
  }
}
int8_t CostmapCore::get(std::vector<int8_t> grid, int res, int y, int x) {
  if (y >= 0 && y < res && x >= 0 && x < res)
    return grid[res * y + x];
  return 100;
}

void CostmapCore::setPose(const nav_msgs::msg::Odometry::SharedPtr odom) {
  tf2::Quaternion quat_tf;
  tf2::fromMsg(odom->pose.pose.orientation, quat_tf);
  double r{}, p{}, y{};
  tf2::Matrix3x3 m(quat_tf);
  m.getRPY(r, p, y);
  robot_angle = y;
  robot_x = odom->pose.pose.position.x;
  robot_y = odom->pose.pose.position.y;
  robot_z = odom->pose.pose.position.z;
}

}