#include <cmath>
#include <map>
#include <string>
#include <vector>

#include "matplotlibcpp.h"
#include "visualization.h"
namespace plt = matplotlibcpp;

namespace ahrs {

void ShowPoints(const std::vector<Point>& points, const std::string& name,
                const std::string& color) {
  std::vector<double> x_list, y_list;
  for (size_t i = 0; i < points.size(); ++i) {
    double cur_x = points[i].x_;
    double cur_y = points[i].y_;
    x_list.push_back(cur_x);
    y_list.push_back(cur_y);
  }
  plt::named_plot(name, x_list, y_list, color);
}

void Visualization::DrawPolygon(const double& x, const double& y,
                                const double& theta, const Config& config) {
  DrawPolygon(x, y, theta, config.tail_, config.front_, config.width_);
}

void Visualization::DrawPolygon(const double& x, const double& y,
                                const double& theta, const double& tail,
                                const double& front, const double& width) {
  /*  y
      ^ p1-------------------------------p2
   *  | .                                .
   *  | .                                .
   *  | .     *(x,y,theta)-->            .
   *  | .                                .
   *  | .                                .
   *  | p4-------------------------------p3
   *  |
   *  ————————————————————————————————————————————>x
   */
  double t = math::NormalizeAngle(theta);
  double half_width = 0.5 * width;
  double p1_x = x + (-tail) * std::cos(t) - half_width * std::sin(t);
  double p1_y = y + (-tail) * std::sin(t) + half_width * std::cos(t);
  double p4_x = x + (-tail) * std::cos(t) - (-half_width) * std::sin(t);
  double p4_y = y + (-tail) * std::sin(t) + (-half_width) * std::cos(t);
  double p2_x = x + front * std::cos(t) - half_width * std::sin(t);
  double p2_y = y + front * std::sin(t) + half_width * std::cos(t);
  double p3_x = x + front * std::cos(t) - (-half_width) * std::sin(t);
  double p3_y = y + front * std::sin(t) + (-half_width) * std::cos(t);
  std::vector<double> x_list = {p1_x, p2_x, p3_x, p4_x};
  std::vector<double> y_list = {p1_y, p2_y, p3_y, p4_y};
  std::map<std::string, std::string> keywords;
  keywords.insert(std::pair<std::string, std::string>("color", "black"));
  plt::fill(x_list, y_list, keywords);
}

void Visualization::ShowResult(const ReferenceLine& line,
                               const Environment& env, const RobotState& state,
                               const Curve& trajectory, const Config& config,
                               const DebugInfo& debug_info) {
  if (line.GetPoints().empty()) {
    return;
  }
  plt::clf();

  // 显示参考线
  ShowPoints(line.GetPoints(), "reference_line", "b--");
  //显示轨迹信息
  ShowPoints(trajectory.Points(), "trajectory", "r-");

  //显示障碍物

  //显示自车
  DrawPolygon(state.x_, state.y_, state.theta_, config);

  plt::axis("equal");
  plt::xlim(state.x_ - 30, state.x_ + 30);
  plt::ylim(state.y_ - 30, state.y_ + 30);
  plt::title("result");
  std::map<std::string, std::string> keywords_label;
  keywords_label.insert(
      std::pair<std::string, std::string>("loc", "upper left"));
  plt::legend(keywords_label);
  plt::pause(0.01);
  // plt::show();  // 显示图表
}
}  // namespace ahrs