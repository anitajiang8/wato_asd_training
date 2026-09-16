#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <vector>
#include <utility>

namespace robot
{

// 2D grid index
struct CellIndex
{
  int x;
  int y;
 
  CellIndex(int xx, int yy) : x(xx), y(yy) {}
  CellIndex() : x(0), y(0) {}
 
  bool operator==(const CellIndex &other) const
  {
    return (x == other.x && y == other.y);
  }
 
  bool operator!=(const CellIndex &other) const
  {
    return (x != other.x || y != other.y);
  }
};

// Hash function for CellIndex so it can be used in std::unordered_map
struct CellIndexHash
{
  std::size_t operator()(const CellIndex &idx) const
  {
    // A simple hash combining x and y
    return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
  }
};


struct AStarNode
{
  CellIndex index;
  double f_score;
  AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};

struct CompareF
{
  bool operator()(const AStarNode &a, const AStarNode &b) { return a.f_score > b.f_score; }
};

class PlannerCore {
  public:
    explicit PlannerCore(const rclcpp::Logger& logger);

    // Runs A* on map from (start_x, start_y) to (goal_x, goal_y), all in world/map-frame meters.
    // Returns a list of world-frame waypoints. Empty if no path exists.
    std::vector<std::pair<double, double>> planPath(
      const nav_msgs::msg::OccupancyGrid &map,
      double start_x, double start_y,
      double goal_x, double goal_y);

  private:
    rclcpp::Logger logger_;

    CellIndex worldToGrid(const nav_msgs::msg::OccupancyGrid &map, double x, double y) const;
    std::pair<double, double> gridToWorld(const nav_msgs::msg::OccupancyGrid &map, const CellIndex &idx) const;
    bool isValid(const nav_msgs::msg::OccupancyGrid &map, const CellIndex &idx) const;
    double heuristic(const CellIndex &a, const CellIndex &b) const;
};

}

#endif