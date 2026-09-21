#include "map_memory_core.hpp"
#include <cmath>

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger)
    : logger_(logger) {
    initializeGlobalMap();
}

void MapMemoryCore::initializeGlobalMap() {
    global_map_.header.frame_id = "sim_world";
    global_map_.info.resolution = resolution_;
    global_map_.info.width = width_;
    global_map_.info.height = height_;
    global_map_.info.origin.position.x = origin_x_;
    global_map_.info.origin.position.y = origin_y_;
    global_map_.info.origin.orientation.w = 1.0;
    global_map_.data.assign(width_ * height_, 0);
}

void MapMemoryCore::integrateCostmap(const nav_msgs::msg::OccupancyGrid& costmap,
                                     double robot_x, double robot_y, double robot_yaw) {
    int cm_width = costmap.info.width;
    int cm_height = costmap.info.height;
    double cm_res = costmap.info.resolution;

  for (int j = 0; j < cm_height; ++j) {
        for (int i = 0; i < cm_width; ++i) {

        int8_t value = costmap.data[j * cm_width + i];

        if (value < 0) continue;

        double local_x = costmap.info.origin.position.x + (i + 0.5) * cm_res;
        double local_y = costmap.info.origin.position.y + (j + 0.5) * cm_res;

        double world_x = robot_x + (local_x * std::cos(robot_yaw) - local_y * std::sin(robot_yaw));
        double world_y = robot_y + (local_x * std::sin(robot_yaw) + local_y * std::cos(robot_yaw));

        int gx = static_cast<int>((world_x - origin_x_) / resolution_);
        int gy = static_cast<int>((world_y - origin_y_) / resolution_);

        if (gx >= 0 && gx < width_ && gy >= 0 && gy < height_) {
            int idx = gy * width_ + gx;
            if (value > global_map_.data[idx]) {
            global_map_.data[idx] = value;
            }
        }
        }
    }
}

const nav_msgs::msg::OccupancyGrid& MapMemoryCore::getGlobalMap() const {
    return global_map_;
}

}
