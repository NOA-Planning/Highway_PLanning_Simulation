#pragma once
#include <limits>
#include <vector>

#include "math/vec2d.h"
namespace ahrs {
using namespace math;
struct Point {
 private:
  /* data */
 public:
  Point(){};
  Point(const double& x, const double& y) : pose_(x, y) {}
  // tips： 这里pose_类的直接初始化方式
  ~Point() {}
  Vec2d pose_;
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
  RobotState() : pose_(0.0, 0.0), theta_(0.0), v_(0.0), w_(0.0), kappa_(0.0) {}
  RobotState(const double& x, const double& y, const double& theta)
      : pose_(x, y), theta_(theta), v_(0.0), w_(0.0), kappa_(0.0) {}
  Vec2d pose_;
  double theta_;
  double v_;
  double w_;
  double kappa_;
};

}  // namespace ahrs