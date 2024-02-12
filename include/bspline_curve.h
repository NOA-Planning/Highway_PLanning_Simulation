#pragma once
#include "curve.h"
namespace ahrs {
class BsplineCurve {
 private:
  /* data */
 public:
  BsplineCurve(/* args */);
  ~BsplineCurve();
  Curve GenerateCurve(const std::vector<Point>& control_points);
};

}  // namespace ahrs
