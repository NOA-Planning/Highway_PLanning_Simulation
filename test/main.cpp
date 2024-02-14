#include <fstream>
#include <iostream>
#include <sstream>

#include "bspline_lattice_planner.h"
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;
using namespace ahrs;
using namespace std;

void UpdateRobotState(RobotState& state, const Curve& trajectory) {
  if (trajectory.Points().empty()) {
    return;
  }
  state.x_ = trajectory.Points().at(1).x_;
  state.y_ = trajectory.Points().at(1).y_;
  state.theta_ = trajectory.Points().at(1).theta_;
}
ReferenceLine LoadRoadInfo(const std::string& file) {
  std::string dir = "/home/ahrs/workspace/nday/bspline_lattice_planner/map/";
  std::string map_name = "berlin_2018.csv";
  std::ifstream road_line_file(dir + map_name);

  std::string line, pose;
  std::getline(road_line_file, line);

  std::vector<Point> points;
  while (getline(road_line_file, line)) {
    std::istringstream s(line);
    getline(s, pose, ',');
    double x = std::atof(pose.c_str());
    getline(s, pose, ',');
    double y = std::atof(pose.c_str());
    getline(s, pose, ',');
    double right_x = x + 1;  // std::atof(pose.c_str());
    getline(s, pose, ',');
    double right_y = y + 1;  // std::atof(pose.c_str());
    getline(s, pose, ',');
    double left_x = x - 1;  // std::atof(pose.c_str());
    getline(s, pose, ',');
    double left_y = y - 1;  // std::atof(pose.c_str());

    Point line_point(x, y);
    line_point.left_boundary_x_ = left_x;
    line_point.left_boundary_y_ = left_y;
    line_point.right_boundary_x_ = right_x;
    line_point.right_boundary_y_ = right_y;
    points.push_back(line_point);
  }

  double s = 0;
  for (size_t i = 0; i + 1 < points.size(); ++i) {
    double theta = math::NormalizeAngle(std::atan2(
        points[i + 1].y_ - points[i].y_, points[i + 1].x_ - points[i].x_));
    points[i].theta_ = theta;
    double ds = std::sqrt(std::pow(points[i + 1].y_ - points[i].y_, 2) +
                          std::pow(points[i + 1].x_ - points[i].x_, 2));
    s += ds;
    points[i + 1].s_ = s;
  }
  ReferenceLine reference_line(points);
  return reference_line;
}

int main() {
  // 指定读取的路线
  std::string dir = "/home/ahrs/workspace/nday/bspline_lattice_planner/map/";
  std::string map_name = "berlin_2018.csv";
  std::string file_name = dir + map_name;
  //读取路线作为参考线
  ReferenceLine reference_line = LoadRoadInfo(file_name);
  if (reference_line.GetPoints().empty()) {
    return 0;
  }
  //感知模块，负责查询障碍物信息
  Environment environment;
  //轨迹规划器
  BsplineLatticePlanner planner;
  //机器人状态
  Point start = reference_line.GetPoints().front();
  RobotState robot_state(start.x_, start.y_, start.theta_);
  //规划结果
  Curve trajectory;
  //可视化模块
  Visualization visualization;
  //车体参数等配置
  Config config;
  DebugInfo debug_info;

  while (true) {
    //更新参考线
    reference_line.Update(robot_state);
    //更新障碍物信息
    environment.Update(robot_state);
    //规划器求解轨迹
    bool res = planner.Plan(robot_state, reference_line, environment,
                            trajectory, config, debug_info);
    //更新机器人的位置（假设机器人能够完美跟踪规划出来的轨迹）
    UpdateRobotState(robot_state, trajectory);
    //可视化当前帧计算结果
    visualization.ShowResult(reference_line, environment, robot_state,
                             trajectory, config, debug_info);
  }

  return 0;
}
