#include "costmap_core.hpp"
#include <cmath>

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger) : logger_(logger) {}

const std::vector<int8_t>& CostmapCore::getCostmapData() const {
    return costmap_data_;
}

int CostmapCore::getWidth() const{
    return width_;
}

int CostmapCore::getHeight() const{
    return height_;
}

double CostmapCore::getResolution() const{
    return resolution_;
}

void CostmapCore::initializeCostmap() {
    costmap_data_.assign(width_ * height_, -1);
}

void CostmapCore::markObstacle(int x_grid, int y_grid) {
    if (x_grid >= 0 && x_grid < width_ && y_grid >= 0 && y_grid < height_) {
        int index = y_grid * width_ + x_grid;
        costmap_data_[index] = 100;
    }
}

void CostmapCore::inflateObstacles() {
    double inflation_radius = 1.0;
    int inflation_cells = inflation_radius / resolution_;
    int max_cost = 100;

    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            if (costmap_data_[y * width_ + x] == 100) {

                for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
                    for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {

                        int nx = x + dx;
                        int ny = y + dy;

                        if (nx >= 0 && nx < width_ && ny >= 0 && ny < height_) {
                            double distance = std::sqrt(dx*dx + dy*dy) * resolution_;

                            if (distance <= inflation_radius) {
                                int cost = max_cost * (1 - distance / inflation_radius);
                                if (cost > costmap_data_[ny * width_ + nx]) {
                                    costmap_data_[ny * width_ + nx] = cost;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

void CostmapCore::convertToGrid(double range, double angle, int& x_grid, int& y_grid) {
    double x_meters = range * std::cos(angle);
    double y_meters = range * std::sin(angle);

    x_grid = static_cast<int>(x_meters / resolution_) + width_ / 2;
    y_grid = static_cast<int>(y_meters / resolution_) + height_ / 2;
}

}
