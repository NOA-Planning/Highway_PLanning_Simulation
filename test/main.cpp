#include <fstream>
#include <iostream>

#include "bspline_lattice_planner.h"
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;
using namespace ahrs;
using namespace std;

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
  ReferenceLine reference_line(points);
  return reference_line;
}

int main() {
  std::string dir = "/home/ahrs/workspace/nday/bspline_lattice_planner/map/";
  std::string map_name = "berlin_2018.csv";
  std::string file_name = dir + map_name;

  ReferenceLine reference_line = LoadRoadInfo(file_name);
  Environment environment;
  Localization fake_localization;
  BsplineLatticePlanner planner;
  RobotState robot_state;
  Curve trajectory;
  Visualization visualization;

  while (true) {
    reference_line.Update();
    environment.Update();
    fake_localization.Update();

    bool res =
        planner.Plan(robot_state, reference_line, environment, trajectory);

    visualization.ShowResult(reference_line, environment, loc, trajectory,
                             config);
  }

  return 0;
}
