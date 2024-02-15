#pragma once

#include "common.h"
#include "config.h"
#include "debug_info.h"
#include "environment.h"
#include "localization.h"
#include "math/math_common.h"
#include "obstacle.h"
#include "reference_line.h"
namespace ahrs {
using namespace math;
class Visualization {
 public:
  void DrawCircle(const double& x, const double& y, const double& radius,
                  const std::string& color);

  void DrawCar(const RobotState& state, const Config& config,
               const std::string& color);
  void DrawPolygon(const double& x, const double& y, const double& theta,
                   const Config& config, const std::string& color,
                   const bool& fill = true);
  void DrawPolygon(const double& x, const double& y, const double& theta,
                   const double& tail, const double& front, const double& width,
                   const std::string& color, const bool& fill = true);
  void DrawPolygon(const Polygon2d& polygon, const std::string& color,
                   const bool& fill = true);
  void ShowPoints(const std::vector<Point>& points, const std::string& name,
                  const std::string& color, const Config& config,
                  const bool& show_car = false);
  void ShowResult(const ReferenceLine& line, const Environment& env,
                  const RobotState& state, const Curve& trajectory,
                  const Config& config, const DebugInfo& debug_info);
  void ShowObstacle(const Environment& env);
};
}  // namespace ahrs