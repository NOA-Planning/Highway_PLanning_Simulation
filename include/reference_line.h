#pragma once
#include <assert.h>

#include "common.h"

namespace ahrs {
class ReferenceLine {
 public:
  ReferenceLine(const std::vector<Point>& points) : points_(points) {}
  void Update(const RobotState& state) {}
  void SetPoints(const std::vector<Point>& points) { points_ = points; }
  const std::vector<Point> GetPoints() const { return points_; }
  const Point GetStart() const {
    // assert(!points_.empty());
    return points_.front();
  }
  const Point GetEnd() const {
    // assert(!points_.empty());
    return points_.back();
  }

  const size_t FindNearestIndex(const double& x, const double& y) const {
    if (points_.empty()) {
      return 0;
    }
    double max_dis = std::numeric_limits<double>::max();
    size_t index = points_.size() - 1;
    double dis = 0;
    for (size_t i = 0; i < points_.size(); ++i) {
      dis = (x - points_[i].x_) * (x - points_[i].x_) +
            (y - points_[i].y_) * (y - points_[i].y_);
      if (dis < max_dis) {
        max_dis = dis;
        index = i;
      }
    }
    return index;
  }

  const size_t FindNearestIndexAtLength(const double& len) const {
    if (points_.empty()) {
      return 0;
    }
    size_t index = points_.size() - 1;
    for (size_t i = 0; i < points_.size(); ++i) {
      if (points_[i].s_ > len) {
        index = i;
        return index;
      }
    }
    return index;
  }

 private:
  std::vector<Point> points_;
};
}  // namespace ahrs