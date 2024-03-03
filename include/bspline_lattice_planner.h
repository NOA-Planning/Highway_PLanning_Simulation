#pragma once
#include "bspline_curve.h"
#include "debug_info.h"
#include "environment.h"
#include "localization.h"
#include "math/math_common.h"
#include "reference_line.h"
#include "trajectory_scorer.h"
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

  std::vector<std::vector<Node2d>> GenerateCtpSequenceByDp(
      const std::vector<std::vector<Vec2d>>& control_point_samples,
      const RobotState& state, const Environment& env);
  std::vector<std::vector<Node2d>> GenerateCtpSequenceByDfs(
      const std::vector<std::vector<Vec2d>>& control_point_samples,
      const RobotState& state, const Environment& env);
  std::vector<CostPath> SamplesScore(
      const std::vector<std::vector<Node2d>>& ctp_seq, const Environment& env,
      const Config& config);
  Curve ChoseBestTrajectory(const std::vector<CostPath>& cost_path);
  DebugInfo debug_info_;
  Config config_;
};
}  // namespace ahrs