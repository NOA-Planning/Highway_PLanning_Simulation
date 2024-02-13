#pragma once
#include "bspline_curve.h"
#include "common.h"
#include "environment.h"
#include "localization.h"
#include "reference_line.h"
#include "visualization.h"
namespace ahrs {

class BsplineLatticePlanner {
 public:
  bool Plan(const RobotState& state, const ReferenceLine& reference_line,
            Environment& env, Curve& trajectory);
};
}  // namespace ahrs