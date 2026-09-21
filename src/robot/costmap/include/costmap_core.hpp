#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace robot
{

class CostmapCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger& logger);

    void initializeCostmap();
    void convertToGrid(double range, double angle, int& x_grid, int& y_grid);
    void markObstacle(int x_grid, int y_grid);
    void inflateObstacles();

    const std::vector<int8_t>& getCostmapData() const;
    int getWidth() const;
    int getHeight() const;
    double getResolution() const;

  private:
    rclcpp::Logger logger_;
    double resolution_ = 0.1;
    int width_ = 300;
    int height_ = 300;
    std::vector<int8_t> costmap_data_;

};


}

#endif
