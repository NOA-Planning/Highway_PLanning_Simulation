#include <fstream>
#include <iostream>
#include "bspline_curve.h"
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;
using namespace std;

int main() {
  std::vector<double> y = {1, 2, 3, 4};
  plt::plot(y);
  plt::show();
  ahrs::BsplineCurve bspline;
  std::vector<ahrs::Point> ctrl_points{{1, 2}, {2, 2}, {3, 3}, {5, 8}, {6, 3}};
  ahrs::Curve curve = bspline.GenerateCurve(ctrl_points);

  const auto points = curve.Points();
  for (int i = 0; i < points.size(); i++) {
    cout << points.at(i).x_ << " " << points.at(i).y_ << std::endl;
  }
  return 0;
}
