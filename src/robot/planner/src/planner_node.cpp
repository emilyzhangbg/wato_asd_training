#include "planner_node.hpp"

#include <vector>

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) 
{
    // Subscribers
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

    // Publisher
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

    // Timer
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10000), std::bind(&PlannerNode::timerCallback, this));

    // Initialize current map to empty grid
    current_map_ = nav_msgs::msg::OccupancyGrid();
    current_map_.header.stamp = this->get_clock()->now();
    current_map_.header.frame_id = "sim_world";
    current_map_.info.width = 30;
    current_map_.info.height = 30;
    current_map_.info.resolution = 1.0;

    // Set origin of the grid
    current_map_.info.origin.position.x = -15.0;
    current_map_.info.origin.position.y = -15.0;
    current_map_.info.origin.orientation.w = 1.0;  // No rotation

    // Create a 1D array representing the grid
    std::vector<int8_t> grid_data(30 * 30, 0);
    current_map_.data = grid_data;
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    current_map_ = *msg;
    if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
        planPath();
    }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    goal_ = *msg;
    goal_received_ = true;
    state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
    planPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    robot_pose_ = msg->pose.pose;
}

void PlannerNode::timerCallback() {
    if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
        if (goalReached()) {
            RCLCPP_INFO(this->get_logger(), "Goal reached!");
            state_ = State::WAITING_FOR_GOAL;
        } else {
            RCLCPP_INFO(this->get_logger(), "Replanning due to timeout or progress...");
            planPath();
        }
    }
}

bool PlannerNode::goalReached() {
    double dx = goal_.point.x - robot_pose_.position.x;
    double dy = goal_.point.y - robot_pose_.position.y;
    return std::sqrt(dx * dx + dy * dy) < 0.5; // Threshold for reaching the goal
}

void PlannerNode::planPath() {
    if (!goal_received_ || current_map_.data.empty()) {
        RCLCPP_WARN(this->get_logger(), "Cannot plan path: Missing map or goal!");
        return;
    }

    nav_msgs::msg::Path path;
    path.header.stamp = this->get_clock()->now();
    path.header.frame_id = "sim_world";

    // Compute path using A* on current_map_
    Grid grid{ current_map_.info.width, current_map_.info.height, current_map_.data.data() };
    // Turn the values to either 0 or 1
    threshold(grid);
    AStar path_finding_algo{ toCellIndex(robot_pose_.position), toCellIndex(goal_.point), grid };
    std::vector<CellIndex> path_points = path_finding_algo.run();

    // Fill path.poses with the resulting waypoints.
    fillPath(path, path_points);
    path_pub_->publish(path);
}

void PlannerNode::fillPath(nav_msgs::msg::Path& path, const std::vector<CellIndex> path_points) {
    for (int i = 0; i < path_points.size(); i++) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = this->get_clock()->now();
        pose.header.frame_id = "sim_world";
        pose.pose.position.x = toCoordPoint(path_points[i]).x;
        pose.pose.position.y = toCoordPoint(path_points[i]).y;
        pose.pose.position.z = toCoordPoint(path_points[i]).z;

        path.poses.push_back(pose);
    }
}

CellIndex PlannerNode::toCellIndex(const geometry_msgs::msg::Point& p) const {
    const geometry_msgs::msg::Point& origin = current_map_.info.origin.position;
    return CellIndex{std::floor((p.x - origin.x) / current_map_.info.resolution), std::floor((p.y - origin.y) / current_map_.info.resolution)};
}

geometry_msgs::msg::Point PlannerNode::toCoordPoint(const CellIndex& p) const {
    const geometry_msgs::msg::Point& origin = current_map_.info.origin.position;
    geometry_msgs::msg::Point out_point;
    out_point.x = (p.x + 0.5) * current_map_.info.resolution + origin.x;
    out_point.y = (p.y + 0.5) * current_map_.info.resolution + origin.y;
    out_point.z = robot_pose_.position.z;

    return out_point;
}

void PlannerNode::threshold(Grid& map, float thresh) {
    for (int y = 0; y < map.height; y++) {
        for (int x = 0; x < map.width; x++) {
            map.data[y * map.width + x] = (map.data[y * map.width + x] > thresh) ? 1 : 0;
        }
    }
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlannerNode>());
    rclcpp::shutdown();
    return 0;
}
