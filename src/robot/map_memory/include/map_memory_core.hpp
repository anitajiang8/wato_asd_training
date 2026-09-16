#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot
{

class MapMemoryCore {
  public:
      explicit MapMemoryCore(const rclcpp::Logger& logger);

      void initializeGlobalMap();

      void integrateCostmap(const nav_msgs::msg::OccupancyGrid& costmap,
                          double robot_x, double robot_y, double robot_yaw);

      const nav_msgs::msg::OccupancyGrid& getGlobalMap() const;

  private:
      rclcpp::Logger logger_;

      nav_msgs::msg::OccupancyGrid global_map_;
      double resolution_ = 0.1;
      int width_ = 600;
      int height_ = 600;
      double origin_x_ = -15.0;
      double origin_y_ = -15.0;
};

}

#endif
