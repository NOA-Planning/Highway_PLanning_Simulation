#pragma once
#include <assert.h>

#include "common.h"

namespace ahrs {
class ReferenceLine {
 public:
  ReferenceLine(const std::vector<Point>& points) : points_(points) {}
  void Update();
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

 private:
  std::vector<Point> points_;
};
}  // namespace ahrs