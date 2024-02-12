#include "bspline_curve.h"

namespace ahrs {
BsplineCurve::BsplineCurve(/* args */) {}
BsplineCurve::~BsplineCurve() {}
Curve BsplineCurve::GenerateCurve(const std::vector<Point>& control_points) {
  Curve curve(control_points);
  return curve;
}

}  // namespace ahrs