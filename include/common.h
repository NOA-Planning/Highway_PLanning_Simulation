#pragma once
#include <vector>

namespace ahrs {

struct Point {
 private:
  /* data */
 public:
  Point(){};
  Point(const double& x, const double& y) : x_(x), y_(y) {}
  ~Point() {}
  double x_;
  double y_;
  double theta_;
  double kappa_;
  double dkappa_;
  double s_;
  double left_boundary_x_;
  double left_boundary_y_;
  double right_boundary_x_;
  double right_boundary_y_;
};

class Curve {
 private:
  std::vector<Point> points_;
  std::vector<Point> control_points_;

 public:
  Curve(){};
  Curve(const std::vector<Point>& ctl_points) : control_points_(ctl_points) {
    points_ = ctl_points;
  }
  ~Curve() {}
  std::vector<Point> Points() const { return points_; }
  std::vector<Point> ControlPoints() const { return control_points_; }
};

struct RobotState {
  double x_;
  double y_;
  double theta_;
  double v_;
  double w_;
  double kappa_;
};

}  // namespace ahrs