#pragma once
#include "math/polygon2d.h"
using namespace ahrs::math;
namespace ahrs {
class Obstacle {
 public:
  Obstacle(const Vec2d& pose, const double& theta, const double& v = 0.0,
           const double& t = 0.0)
      : pose_(pose), theta_(theta), v_(v), t_(t) {
    UpdatePolygon();
  };
  void SetPose(const Vec2d& pose) {
    pose_ = pose;
    UpdatePolygon();
  }
  void SetTheta(const double& theta) {
    theta_ = theta;
    UpdatePolygon();
  }
  void SetV(const double& v) { v_ = v; }
  void SetTime(const double& time) { t_ = time; }
  const Vec2d GetPose() const { return pose_; }
  const double GetTheta() const { return theta_; }

  const double GetV() const { return v_; }
  const double GetT() const { return t_; }
  const Polygon2d GetPolygon() const { return polygon_; }

 private:
  void UpdatePolygon() {
    Box2d box(pose_, theta_, 5, 2);
    polygon_ = Polygon2d(box);
  }
  Vec2d pose_;
  double theta_;
  double v_;
  double t_;
  Polygon2d polygon_;
};
}  // namespace ahrs