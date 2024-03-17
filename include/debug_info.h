#pragma once
#include "common.h"
namespace ahrs {
struct DebugInfo {
  std::vector<std::vector<Vec2d>> sample_control_points_;
  std::vector<Point> reference_line_points_;
  std::vector<std::vector<Node2d>> ctp_sequence_;
  std::vector<std::vector<Point>> bspline_samples_;
};

}  // namespace ahrs