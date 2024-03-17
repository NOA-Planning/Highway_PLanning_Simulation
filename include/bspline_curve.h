#pragma once
#include <Eigen/Core>

#include "common.h"

using namespace Eigen;
namespace ahrs {
class BsplineCurve {
 private:
  /* data */
 public:
  BsplineCurve() {}
  BsplineCurve(const double& interval = 0.001);
  ~BsplineCurve();
  std::vector<Point> GenerateCurve();
  void SetControlPoints(const std::vector<Vec2d>& control_points);

 private:
  Point GetPos(const size_t& k, const double& t);

  double interval_;
  std::vector<Vec2d> ctp_;
  std::vector<Vec2d> pos_;
  size_t ctp_size_;
};

}  // namespace ahrs
