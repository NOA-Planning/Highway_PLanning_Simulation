#pragma once
#include "bspline_curve.h"
#include "debug_info.h"
#include "environment.h"
#include "localization.h"
#include "math/math_common.h"
#include "reference_line.h"
#include "visualization.h"
namespace ahrs {

class BsplineLatticePlanner {
 public:
  bool Plan(const RobotState& state, const ReferenceLine& reference_line,
            Environment& env, Curve& trajectory, const Config& config);
  DebugInfo GetDebugInfo() const { return debug_info_; }

 private:
  void SampleControlPoints(
      const RobotState& state, const ReferenceLine& reference_line,
      Environment& env, std::vector<std::vector<Vec2d>>& control_point_samples);

  std::vector<std::vector<Vec2d>> GenerateCtpSequenceByDp(
      const std::vector<std::vector<Vec2d>>& control_point_samples,
      const RobotState& state, const Environment& env);
  std::vector<std::vector<Vec2d>> GenerateCtpSequenceByDfs(
      const std::vector<std::vector<Vec2d>>& control_point_samples,
      const RobotState& state, const Environment& env);

  DebugInfo debug_info_;
  Config config_;
};
}  // namespace ahrs