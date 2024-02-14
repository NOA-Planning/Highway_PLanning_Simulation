#include "bspline_lattice_planner.h"
namespace ahrs {

bool BsplineLatticePlanner::Plan(const RobotState& state,
                                 const ReferenceLine& reference_line,
                                 Environment& env, Curve& trajectory,
                                 const Config& config, DebugInfo& debug_info) {
  size_t near_index = reference_line.FindNearestIndex(state.x_, state.y_);
  std::vector<Point> points = reference_line.GetPoints();
  double s = points[near_index].s_ + 20.0;
  size_t end_index = reference_line.FindNearestIndexAtLength(s);
  std::vector<Point> new_points(points.begin() + near_index,
                                points.begin() + end_index + 1);
  Curve trajectory_temp(new_points);
  trajectory = trajectory_temp;

  return true;
}
}  // namespace ahrs
