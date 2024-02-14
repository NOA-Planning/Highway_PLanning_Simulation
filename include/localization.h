#pragma once
namespace ahrs {
class Localization {
 public:
  void Update(const RobotState& state, const Curve& trajectory) {}
};
}  // namespace ahrs