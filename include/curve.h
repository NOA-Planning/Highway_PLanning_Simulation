#pragma once
#include <vector>

namespace ahrs {

struct Point {
 private:
  /* data */
 public:
  Point(const double& x, const double& y) : x_(x), y_(y) {}
  ~Point() {}
  double x_;
  double y_;
};

class Curve {
 private:
  std::vector<Point> points_;
  std::vector<Point> control_points_;

 public:
  Curve(const std::vector<Point>& ctl_points) : control_points_(ctl_points) {
    points_ = ctl_points;
  }
  ~Curve() {}
  std::vector<Point> Points() const { return points_; }
  std::vector<Point> ControlPoints() const { return control_points_; }
};

}  // namespace ahrs