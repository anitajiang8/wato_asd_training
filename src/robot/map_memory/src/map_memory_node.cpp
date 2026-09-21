#include <chrono>
#include <cmath>
#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode()
  : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {

      costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/costmap", 10,
      std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));

      odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10,
      std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

      map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

      timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&MapMemoryNode::updateMap, this));
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    latest_costmap_ = *msg;
    costmap_received_ = true;
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    robot_x_ = msg->pose.pose.position.x;
    robot_y_ = msg->pose.pose.position.y;

    double qx = msg->pose.pose.orientation.x;
    double qy = msg->pose.pose.orientation.y;
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;
    robot_yaw_ = std::atan2(2.0 * (qw * qz + qx * qy),
                          1.0 - 2.0 * (qy * qy + qz * qz));

    double dist = std::sqrt(std::pow(robot_x_ - last_x_, 2) +
                          std::pow(robot_y_ - last_y_, 2));
    if (dist >= distance_threshold_) {
        last_x_ = robot_x_;
        last_y_ = robot_y_;
        should_update_ = true;
    }
}

void MapMemoryNode::updateMap() {
    if (should_update_ && costmap_received_) {
        map_memory_.integrateCostmap(latest_costmap_, robot_x_, robot_y_, robot_yaw_);
        map_pub_->publish(map_memory_.getGlobalMap());
        should_update_ = false;
    }
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapMemoryNode>());
    rclcpp::shutdown();
    return 0;
}
