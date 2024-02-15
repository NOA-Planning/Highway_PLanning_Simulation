#include <cmath>
#include <map>
#include <string>
#include <vector>

#include "matplotlibcpp.h"
#include "visualization.h"
namespace plt = matplotlibcpp;
namespace ahrs {
using namespace math;
// tips:  const 对象上调用了一个非 const
// 成员函数,这里入参的env是const对象，而StaticObstacle()是非const成员函数，导致报错。
// 这里需要将StaticObstacle修改为const函数
void Visualization::ShowObstacle(const Environment& env) {
  for (const auto& obs : env.StaticObstacle()) {
    DrawPolygon(obs.GetPolygon(), "green");
  }

  for (const auto& obs : env.DynamicObstacle()) {
    DrawPolygon(obs.GetPolygon(), "pink");
  }
}

void Visualization::ShowPoints(const std::vector<Point>& points,
                               const std::string& name,
                               const std::string& color, const Config& config,
                               const bool& show_car) {
  std::vector<double> x_list, y_list;
  for (size_t i = 0; i < points.size(); ++i) {
    double cur_x = points[i].pose_.x();
    double cur_y = points[i].pose_.y();
    x_list.push_back(cur_x);
    y_list.push_back(cur_y);
    if (show_car) {
      DrawPolygon(cur_x, cur_y, points[i].theta_, config, "cyan", false);
    }
  }
  plt::named_plot(name, x_list, y_list, color);
}

void Visualization::DrawCircle(const double& x, const double& y,
                               const double& radius, const std::string& color) {
  std::vector<double> x_list, y_list;

  for (double i = 0; i < 360; i += 1) {  // 以1度为间隔生成圆的点
    x_list.push_back(x + radius * cos(i * M_PI / 180));  // 计算圆上点的x坐标
    y_list.push_back(y + radius * sin(i * M_PI / 180));  // 计算圆上点的y坐标
  }
  plt::plot(x_list, y_list, color);
}

void Visualization::DrawCar(const RobotState& state, const Config& config,
                            const std::string& color) {
  // 1. 车体
  DrawPolygon(state.pose_.x(), state.pose_.y(), state.theta_, config, color);
  // 2. 车轮
  double tail = config.tail_ - 0.3;
  double front = config.front_ - 0.3;
  double half_width = config.width_ * 0.5 - 0.1;
  // 左后
  double p1_x = state.pose_.x() + (-tail) * std::cos(state.theta_) -
                half_width * std::sin(state.theta_);
  double p1_y = state.pose_.y() + (-tail) * std::sin(state.theta_) +
                half_width * std::cos(state.theta_);
  DrawPolygon(p1_x, p1_y, state.theta_, 0.3, 0.3, 0.2, "black");
  // 右后
  double p4_x = state.pose_.x() + (-tail) * std::cos(state.theta_) -
                (-half_width) * std::sin(state.theta_);
  double p4_y = state.pose_.y() + (-tail) * std::sin(state.theta_) +
                (-half_width) * std::cos(state.theta_);
  DrawPolygon(p4_x, p4_y, state.theta_, 0.3, 0.3, 0.2, "black");

  // 左前
  double p2_x = state.pose_.x() + front * std::cos(state.theta_) -
                half_width * std::sin(state.theta_);
  double p2_y = state.pose_.y() + front * std::sin(state.theta_) +
                half_width * std::cos(state.theta_);
  DrawPolygon(p2_x, p2_y, state.theta_, 0.3, 0.3, 0.2, "black");

  // 右前
  double p3_x = state.pose_.x() + front * std::cos(state.theta_) -
                (-half_width) * std::sin(state.theta_);
  double p3_y = state.pose_.y() + front * std::sin(state.theta_) +
                (-half_width) * std::cos(state.theta_);
  DrawPolygon(p3_x, p3_y, state.theta_, 0.3, 0.3, 0.2, "black");

  // 3.方向盘
  double center_x = 0.5 * (p2_x + p3_x);
  double center_y = 0.5 * (p2_y + p3_y);
  DrawCircle(center_x, center_y, 0.3, "k-");
  DrawCircle(center_x, center_y, 0.25, "k-");
}

void Visualization::DrawPolygon(const double& x, const double& y,
                                const double& theta, const Config& config,
                                const std::string& color, const bool& fill) {
  DrawPolygon(x, y, theta, config.tail_, config.front_, config.width_, color,
              fill);
}

void Visualization::DrawPolygon(const double& x, const double& y,
                                const double& theta, const double& tail,
                                const double& front, const double& width,
                                const std::string& color, const bool& fill) {
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
  std::vector<double> x_list = {p1_x, p2_x, p3_x, p4_x, p1_x};
  std::vector<double> y_list = {p1_y, p2_y, p3_y, p4_y, p1_y};
  if (fill) {
    std::map<std::string, std::string> keywords;
    keywords.insert(std::pair<std::string, std::string>("color", color));
    plt::fill(x_list, y_list, keywords);

  } else {
    plt::plot(x_list, y_list, color);
  }
}

void Visualization::DrawPolygon(const Polygon2d& polygon,
                                const std::string& color, const bool& fill) {
  std::vector<double> x, y;
  for (const auto& p : polygon.points()) {
    x.push_back(p.x());
    y.push_back(p.y());
  }
  x.push_back(x.front());
  y.push_back(y.front());
  if (fill) {
    std::map<std::string, std::string> keywords;
    keywords.insert(std::pair<std::string, std::string>("color", color));
    plt::fill(x, y, keywords);

  } else {
    plt::plot(x, y, color);
  }
}

void Visualization::ShowResult(const ReferenceLine& line,
                               const Environment& env, const RobotState& state,
                               const Curve& trajectory, const Config& config,
                               const DebugInfo& debug_info) {
  if (line.GetPoints().empty()) {
    return;
  }
  plt::clf();
  plt::subplot(2, 1, 1);
  // 显示参考线
  ShowPoints(line.GetPoints(), "reference_line", "b--", config);
  //显示轨迹信息
  ShowPoints(trajectory.Points(), "trajectory", "r-", config);

  //显示障碍物
  ShowObstacle(env);

  //显示自车
  DrawCar(state, config, "grey");

  plt::axis("equal");
  plt::xlim(state.pose_.x() - config.length_, state.pose_.x() + config.length_);
  plt::ylim(state.pose_.y() - config.length_, state.pose_.y() + config.length_);
  plt::title("result");
  std::map<std::string, std::string> keywords_label;
  keywords_label.insert(
      std::pair<std::string, std::string>("loc", "upper left"));
  plt::legend(keywords_label);

  //显示文字信息
  // 计算右上角的坐标位置，留出一些边距
  double x_text = plt::xlim()[1] - (plt::xlim()[1] - plt::xlim()[0]) * 0.1;
  double y_text = plt::ylim()[1] - (plt::ylim()[1] - plt::ylim()[0]) * 0.05;
  // 在右上角显示文字信息
  size_t index = line.FindNearestIndex(state.pose_.x(), state.pose_.y());
  double s = line.GetPoints()[index].s_;
  std::string s1 = "s: " + std::to_string(s);
  std::string s2 = "pose:  " + std::to_string(state.pose_.x()) + " , " +
                   std::to_string(state.pose_.y()) + " , " +
                   std::to_string(state.theta_);
  std::string s3 = "v: " + std::to_string(state.v_);

  plt::text(x_text, y_text, s1);
  plt::text(x_text, y_text - 2, s2);
  plt::text(x_text, y_text - 4, s3);

  plt::subplot(2, 1, 2);
  // tips:直接subplot会报错，
  // PyTuple_SetItem(args, 0, PyFloat_FromDouble(nrows));
  // 修改为PyTuple_SetItem(args, 0, PyLong_FromDouble(nrows));
  ShowPoints(trajectory.Points(), "trajectory", "r-", config, true);
  DrawCar(state, config, "grey");
  plt::axis("equal");
  plt::title("trajectory");

  plt::pause(0.01);
  // plt::show();  // 显示图表
}
}  // namespace ahrs