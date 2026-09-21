#include <chrono>
#include <memory>

#include "costmap_node.hpp"

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
    // Subscriber: /lidar, LaserScan
    lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/lidar", 10,
        std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));

    // Publisher: /costmap, OccupancyGrid
    costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    // Step 1: Initialize costmap
    costmap_.initializeCostmap();

    // Step 2: Convert LaserScan to grid and mark obstacles
    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        double angle = scan->angle_min + i * scan->angle_increment;
        double range = scan->ranges[i];
        if (range < scan->range_max && range > scan->range_min) {
            int x_grid, y_grid;
            costmap_.convertToGrid(range, angle, x_grid, y_grid);
            costmap_.markObstacle(x_grid, y_grid);
        }
    }

    // Step 3: Inflate obstacles
    costmap_.inflateObstacles();

    // Step 4: Publish costmap
    nav_msgs::msg::OccupancyGrid msg;
    msg.header.stamp = scan->header.stamp;
    msg.header.frame_id = "robot/chassis/lidar";

    msg.info.resolution = costmap_.getResolution();
    msg.info.width = costmap_.getWidth();
    msg.info.height = costmap_.getHeight();

    msg.info.origin.position.x = -(costmap_.getWidth() * costmap_.getResolution()) / 2.0;
    msg.info.origin.position.y = -(costmap_.getHeight() * costmap_.getResolution()) / 2.0;
    msg.info.origin.orientation.w = 1.0;

    msg.data = costmap_.getCostmapData();

    costmap_pub_->publish(msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
