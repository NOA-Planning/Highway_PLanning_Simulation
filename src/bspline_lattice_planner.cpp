#include "bspline_lattice_planner.h"
#include "dp_path.h"
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

  std::vector<std::vector<Vec2d>> ctp_sequence =
      GenerateCtpSequence(control_point_samples, state, env);

  trajectory = Curve(debug_info_.reference_line_points_);

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
         dy < config_.sample_half_width_; dy += config_.ctp_interval_y_) {
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
std::vector<std::vector<Vec2d>> BsplineLatticePlanner::GenerateCtpSequence(
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
  std::vector<Vec2d> path = dp_path.DpSearch();

  debug_info_.ctp_sequence_ = {path};

  return {path};
}
}  // namespace ahrs
