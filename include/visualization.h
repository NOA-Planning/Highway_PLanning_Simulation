#pragma once

#include "common.h"
#include "config.h"
#include "debug_info.h"
#include "environment.h"
#include "localization.h"
#include "math/math_common.h"
#include "reference_line.h"
namespace ahrs {
class Visualization {
 public:
  void DrawPolygon(const double& x, const double& y, const double& theta,
                   const Config& config);
  void DrawPolygon(const double& x, const double& y, const double& theta,
                   const double& tail, const double& front,
                   const double& width);
  void ShowResult(const ReferenceLine& line, const Environment& env,
                  const RobotState& state, const Curve& trajectory,
                  const Config& config, const DebugInfo& debug_info);
};
}  // namespace ahrs