#include <algorithm>
#include <queue>
#include <set>

#include "bspline_lattice_planner.h"
#include "dp_path.h"
#include "grid_graph.h"
namespace ahrs {

bool BsplineLatticePlanner::Plan(const RobotState& state,
                                 const ReferenceLine& reference_line,
                                 Environment& env, Curve& trajectory,
                                 const Config& config) {
  config_ = config;
  std::vector<std::vector<Vec2d>> control_point_samples;
  //控制点采样
  SampleControlPoints(state, reference_line, env, control_point_samples);
  // DP遍历

  // std::vector<std::vector<Vec2d>> ctp_sequence =
  //     GenerateCtpSequenceByDp(control_point_samples, state, env);

  std::vector<std::vector<Node2d>> ctp_sequence =
      GenerateCtpSequenceByDfs(control_point_samples, state, env);

  std::vector<CostPath> sort_vec = SamplesScore(ctp_sequence, env, config);

  trajectory = ChoseBestTrajectory(sort_vec);

  return true;
}
void BsplineLatticePlanner::SampleControlPoints(
    const RobotState& state, const ReferenceLine& reference_line,
    Environment& env, std::vector<std::vector<Vec2d>>& control_point_samples) {
  // tips:采样个数如果确定，可以提前分配数组内存，加速计算
  std::vector<Point> reference_points = reference_line.GetPoints();
  size_t start_index =
      reference_line.FindNearestIndex(state.pose_.x(), state.pose_.y());
  double end_s = reference_points.at(start_index).s_ + config_.sample_length_;
  size_t end_index = reference_line.FindNearestIndexAtLength(end_s);
  Point last_point = reference_points.at(start_index);
  for (size_t i = start_index; i <= end_index; ++i) {
    Point center_point = reference_points.at(i);
    if (center_point.s_ - last_point.s_ < config_.ctp_interval_x_) {
      continue;
    }
    double theta = center_point.theta_;
    //从左到右按照间隔采样
    std::vector<Vec2d> one_layer_sample;
    for (double dy = -config_.sample_half_width_;
         dy < config_.sample_half_width_ + 0.001;
         dy += config_.ctp_interval_y_) {
      double sample_x =
          center_point.pose_.x() + 0 * std::cos(theta) - dy * std::sin(theta);
      double sample_y =
          center_point.pose_.y() + 0 * std::sin(theta) + dy * cos(theta);
      Vec2d left_ctp(sample_x, sample_y);
      one_layer_sample.emplace_back(Vec2d(sample_x, sample_y));
    }

    control_point_samples.push_back(one_layer_sample);
    last_point = center_point;
  }
  // debug info
  debug_info_.sample_control_points_ = control_point_samples;
  debug_info_.reference_line_points_.assign(
      reference_points.begin() + start_index,
      reference_points.begin() + end_index + 1);
}
std::vector<std::vector<Node2d>> BsplineLatticePlanner::GenerateCtpSequenceByDp(
    const std::vector<std::vector<Vec2d>>& control_point_samples,
    const RobotState& state, const Environment& env) {
  //规划起点
  Vec2d start_ctp(state.pose_.x(), state.pose_.y());
  double theta = state.theta_;
  double x = start_ctp.x() + config_.zero_layer_interval_ * std::cos(theta) -
             0 * std::sin(theta);
  double y = start_ctp.y() + config_.zero_layer_interval_ * std::sin(theta) +
             0 * cos(theta);
  Vec2d zero_ctp(x, y);

  // tips: 这里可以用std::move
  // tips:vector的各种初始化构造讲一下

  std::vector<std::vector<Vec2d>> ctp_samples = {{zero_ctp}};
  // tips:insert如何插在Vetor前边，比插后边更耗时
  ctp_samples.insert(ctp_samples.end(), control_point_samples.begin(),
                     control_point_samples.end());

  DpPath dp_path(ctp_samples, env, config_);
  std::vector<Node2d> path = dp_path.DpSearch();

  debug_info_.ctp_sequence_ = {path};

  return {path};
}
std::vector<std::vector<Node2d>>
BsplineLatticePlanner::GenerateCtpSequenceByDfs(
    const std::vector<std::vector<Vec2d>>& control_point_samples,
    const RobotState& state, const Environment& env) {
  //规划起点
  Vec2d start_ctp(state.pose_.x(), state.pose_.y());
  double theta = state.theta_;
  double x = start_ctp.x() + config_.zero_layer_interval_ * std::cos(theta) -
             0 * std::sin(theta);
  double y = start_ctp.y() + config_.zero_layer_interval_ * std::sin(theta) +
             0 * cos(theta);
  Vec2d zero_ctp(x, y);

  // tips: 这里可以用std::move
  // tips:vector的各种初始化构造讲一下
  // std::vector<std::vector<Vec2d>> ctp_samples = {{start_ctp}, {zero_ctp}};
  std::vector<std::vector<Vec2d>> ctp_samples;
  ctp_samples.push_back({start_ctp});
  ctp_samples.push_back({zero_ctp});

  // tips:insert如何插在Vetor前边，比插后边更耗时
  ctp_samples.insert(ctp_samples.end(), control_point_samples.begin(),
                     control_point_samples.end());
  GridGraph graph(ctp_samples, env, config_);
  // std::vector<std::vector<Node2d>> all_ctp_sequence =
  //     graph.GraphSearchWithStack();
  std::vector<std::vector<Node2d>> all_ctp_sequence = graph.GraphSearch();
  debug_info_.ctp_sequence_ = all_ctp_sequence;

  return all_ctp_sequence;
}
std::vector<CostPath> BsplineLatticePlanner::SamplesScore(
    const std::vector<std::vector<Node2d>>& ctp_seq, const Environment& env,
    const Config& config) {
  std::vector<CostPath> sort_vec;
  TrajectoryScorer scorer;
  double cost = 0.0;
  for (auto& seq : ctp_seq) {
    cost = scorer.GetScorer(seq, env, config);
    sort_vec.emplace_back(CostPath(cost, seq));
  }

  std::sort(sort_vec.begin(), sort_vec.end(), CostPath::CompareCost());
  return sort_vec;
}
Curve BsplineLatticePlanner::ChoseBestTrajectory(
    const std::vector<CostPath>& cost_path) {
  if (cost_path.empty()) {
    return Curve();
  }

  CostPath best_ctp = cost_path.front();
  std::vector<Point> best_trajectory;
  for (size_t i = 0; i + 1 < best_ctp.path_.size(); ++i) {
    Vec2d cur = best_ctp.path_[i].pos_;
    Vec2d next = best_ctp.path_[i + 1].pos_;
    Vec2d temp = next - cur;
    Point point(cur.x(), cur.y());
    point.theta_ = temp.Angle();
    best_trajectory.emplace_back(point);
  }

  //最后一个点
  Point last_point(best_ctp.path_.back().pos_.x(),
                   best_ctp.path_.back().pos_.y());
  last_point.theta_ = best_trajectory.back().theta_;
  best_trajectory.emplace_back(last_point);
  return best_trajectory;
}
}  // namespace ahrs
