#include "planner_core.hpp"
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <cmath>

namespace robot
{

PlannerCore::PlannerCore(const rclcpp::Logger& logger) : logger_(logger) {}

CellIndex PlannerCore::worldToGrid(const nav_msgs::msg::OccupancyGrid &map, double x, double y) const {
  int gx = static_cast<int>((x - map.info.origin.position.x) / map.info.resolution);
  int gy = static_cast<int>((y - map.info.origin.position.y) / map.info.resolution);
  return CellIndex(gx, gy);
}

std::pair<double, double> PlannerCore::gridToWorld(const nav_msgs::msg::OccupancyGrid &map, const CellIndex &idx) const {
  double x = map.info.origin.position.x + (idx.x + 0.5) * map.info.resolution;
  double y = map.info.origin.position.y + (idx.y + 0.5) * map.info.resolution;
  return {x, y};
}

bool PlannerCore::isValid(const nav_msgs::msg::OccupancyGrid &map, const CellIndex &idx) const {
  if (idx.x < 0 || idx.y < 0) return false;
  if (idx.x >= static_cast<int>(map.info.width) || idx.y >= static_cast<int>(map.info.height)) return false;

  int flat_index = idx.y * map.info.width + idx.x;
  int8_t value = map.data[flat_index];

  const int8_t OCCUPIED_THRESHOLD = 50;
  return value < OCCUPIED_THRESHOLD;
}

double PlannerCore::heuristic(const CellIndex &a, const CellIndex &b) const {
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

std::vector<std::pair<double, double>> PlannerCore::planPath(
    const nav_msgs::msg::OccupancyGrid &map,
    double start_x, double start_y,
    double goal_x, double goal_y)
{
  CellIndex start = worldToGrid(map, start_x, start_y);
  CellIndex goal = worldToGrid(map, goal_x, goal_y);

  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;
  std::unordered_map<CellIndex, double, CellIndexHash> g_score;
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
  std::unordered_set<CellIndex, CellIndexHash> closed_set;

  g_score[start] = 0.0;
  open_set.push(AStarNode(start, heuristic(start, goal)));

  const std::vector<std::tuple<int, int, double>> directions = {
    {1, 0, 1.0}, {-1, 0, 1.0}, {0, 1, 1.0}, {0, -1, 1.0},
    {1, 1, std::sqrt(2.0)}, {1, -1, std::sqrt(2.0)}, {-1, 1, std::sqrt(2.0)}, {-1, -1, std::sqrt(2.0)}
  };

  while (!open_set.empty()) {
    CellIndex current = open_set.top().index;
    open_set.pop();

    if (current == goal) {
      std::vector<CellIndex> cell_path;
      CellIndex c = current;
      while (!(c == start)) {
        cell_path.push_back(c);
        c = came_from[c];
      }
      cell_path.push_back(start);
      std::reverse(cell_path.begin(), cell_path.end());

      std::vector<std::pair<double, double>> world_path;
      for (const auto &cell : cell_path) {
        world_path.push_back(gridToWorld(map, cell));
      }
      return world_path;
    }

    if (closed_set.count(current)) continue;
    closed_set.insert(current);

    for (const auto &[dx, dy, cost] : directions) {
      CellIndex neighbor(current.x + dx, current.y + dy);
      if (!isValid(map, neighbor) || closed_set.count(neighbor)) continue;

      double tentative_g = g_score[current] + cost;
      if (!g_score.count(neighbor) || tentative_g < g_score[neighbor]) {
        g_score[neighbor] = tentative_g;
        came_from[neighbor] = current;
        open_set.push(AStarNode(neighbor, tentative_g + heuristic(neighbor, goal)));
      }
    }
  }

  RCLCPP_WARN(logger_, "No path found from start to goal");
  return {};
}

}