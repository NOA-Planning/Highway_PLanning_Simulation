
#include "environment.h"

namespace ahrs {
Environment::Environment(const ReferenceLine& reference_line,
                         const Config& config)
    : reference_line_(reference_line), config_(config) {
  GenerateObstacle();
}
// tips:这里初始化列表，直接初始化reference_line_等成员变量，属于拷贝构造，对于参考线类和config类，会自动生成拷贝构造函数，但是这里为什么会报错呢？
//原来是config_声明的时候有问题，写错成了Config& config_;
void Environment::Update(const RobotState& state) {
  //更新动态障碍物的位置

  constexpr double max_window_length = 50.0;
  double dt = config_.interval_;
  for (auto& obs : dynamic_obstacles_) {
    double dis = state.pose_.DistanceTo(obs.GetPose());
    if (dis > max_window_length) {
      continue;
    }
    double next_x =
        obs.GetPose().x() + obs.GetV() * dt * std::cos(obs.GetTheta());
    double next_y =
        obs.GetPose().y() + obs.GetV() * dt * std::sin(obs.GetTheta());
    Vec2d new_pose(next_x, next_y);
    obs.SetPose(new_pose);
  }
}

void Environment::GenerateObstacle() {
  // 1.沿着参考线生成静态障碍物
  double last_s = 0;
  for (const auto& p : reference_line_.GetPoints()) {
    if (p.s_ - last_s > 30.0) {
      Obstacle obs(p.pose_, p.theta_);
      static_obstacles_.push_back(obs);

      Vec2d new_pose(p.pose_.x() + 2.0, p.pose_.y() + 3.0);
      Obstacle obs1(new_pose, p.theta_ + M_PI_2);
      static_obstacles_.push_back(obs1);

      Vec2d new_pose1(p.pose_.x() + 10.0, p.pose_.y() - 10.0);
      Obstacle obs2(new_pose1, p.theta_ + M_PI_4);
      static_obstacles_.push_back(obs2);
      last_s = p.s_;
    }
  }

  last_s = 0;
  // 2.沿着参考线生成动态障碍物
  for (const auto& p : reference_line_.GetPoints()) {
    if (p.s_ - last_s < 50.0) {
      continue;
    }
    if (p.s_ < 50.0) {
      Obstacle obs(p.pose_, p.theta_, 5.0, 0.0);
      dynamic_obstacles_.push_back(obs);
      Vec2d pose_new(p.pose_ + Vec2d(15.0, 5.0));
      obs.SetPose(pose_new);
      dynamic_obstacles_.push_back(obs);
      last_s = p.s_;
    } else if (p.s_ < 100.0) {
      Obstacle obs(p.pose_, p.theta_ + M_PI, 2.0, 0.0);
      dynamic_obstacles_.push_back(obs);
      last_s = p.s_;
    } else if (p.s_ < 150.0) {
      Obstacle obs(p.pose_, p.theta_ + M_PI_2, 13.0, 0.0);
      dynamic_obstacles_.push_back(obs);
      Vec2d pose_new(p.pose_ + Vec2d(1.0, 2.0));
      obs.SetPose(pose_new);
      dynamic_obstacles_.push_back(obs);
      last_s = p.s_;
    } else if (p.s_ < 200.0) {
      Obstacle obs(p.pose_, p.theta_ + M_PI_4, 10.0, 0.0);
      dynamic_obstacles_.push_back(obs);
      last_s = p.s_;
    } else if (p.s_ < 250.0) {
      Obstacle obs(p.pose_, p.theta_, 2.0, 0.0);
      dynamic_obstacles_.push_back(obs);
      last_s = p.s_;
    } else if (p.s_ < 300.0) {
      Obstacle obs(p.pose_, p.theta_, 2.0, 0.0);
      dynamic_obstacles_.push_back(obs);
      last_s = p.s_;
    } else {
      Obstacle obs(p.pose_, p.theta_, 2.0, 0.0);
      dynamic_obstacles_.push_back(obs);
      last_s = p.s_;
    }
  }
}

std::vector<Obstacle> Environment::StaticObstacle() const {
  return static_obstacles_;
}

std::vector<Obstacle> Environment::DynamicObstacle() const {
  return dynamic_obstacles_;
}

}  // namespace ahrs
