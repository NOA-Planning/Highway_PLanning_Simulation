#include "trajectory_scorer.h"
namespace ahrs {

double TrajectoryScorer::GetScorer(const std::vector<Node2d>& path,
                                   const Environment& env,
                                   const Config& config) {
  double score_smooth = SmoothTerm(path, env, config);
  double score_safe = SafeTerm(path, env, config);
  double score_traffic = TrafficTerm(path, env, config);
  double score_curvature = CurvatureTerm(path, env, config);
  double score_consistency = ConsistencyTerm(path, env, config);
  double cost = score_smooth + score_safe + score_traffic + score_curvature +
                score_consistency;
  return cost;
}

double TrajectoryScorer::SmoothTerm(const std::vector<Node2d>& path,
                                    const Environment& env,
                                    const Config& config) {
  return 0.0;
}
double TrajectoryScorer::SafeTerm(const std::vector<Node2d>& path,
                                  const Environment& env,
                                  const Config& config) {
  return 0.0;
}
double TrajectoryScorer::TrafficTerm(const std::vector<Node2d>& path,
                                     const Environment& env,
                                     const Config& config) {
  double score = 0.0;
  for (const auto& p : path) {
    score += static_cast<double>(p.index_.j_);
  }
  return score;
}
double TrajectoryScorer::CurvatureTerm(const std::vector<Node2d>& path,
                                       const Environment& env,
                                       const Config& config) {
  return 0.0;
}
double TrajectoryScorer::ConsistencyTerm(const std::vector<Node2d>& path,
                                         const Environment& env,
                                         const Config& config) {
  return 0.0;
}
}  // namespace ahrs
