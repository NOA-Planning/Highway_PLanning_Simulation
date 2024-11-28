#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

int main() {
  std::vector<double> x_list, y_list, left_boundary_x, left_boundary_y,
      right_boundary_x, right_boundary_y;
  std::string dir = "../map/";
  std::string map_name = "berlin_2018.csv";
  std::ifstream road_line_file(dir + map_name);

  std::string line, pose;
  std::getline(road_line_file, line);
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

    x_list.emplace_back(x);
    y_list.push_back(y);
    left_boundary_x.push_back(left_x);
    left_boundary_y.push_back(left_y);

    right_boundary_x.push_back(right_x);
    right_boundary_y.push_back(right_y);
  }

  // 在循环外绘图
  plt::named_plot("center-line", x_list, y_list, "k-");
  plt::named_plot("l-boundary", left_boundary_x, left_boundary_y, "b-");
  plt::named_plot("r-boundary", right_boundary_x, right_boundary_y, "b-");

  // 显示图例
  plt::legend();

  plt::title(map_name);

  plt::show();  // 显示图表

  return 0;
}
