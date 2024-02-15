#pragma once
#include "config.h"
#include "obstacle.h"
#include "reference_line.h"
namespace ahrs {
class Environment {
 public:
  Environment(const ReferenceLine& reference_line, const Config& config);
  // tip:将A类作为B类的构造函数参数，A类的构造函数有什么要求
  void Update(const RobotState& state);
  void GenerateObstacle();
  std::vector<Obstacle> StaticObstacle() const;
  std::vector<Obstacle> DynamicObstacle() const;
  // tips:
  //当const关键字用于修饰函数的返回值时，它限制了返回值的修改。这主要用于引用或指针的返回类型，以确保返回的数据不能通过这个引用或指针被修改。
  //当const关键字用于成员函数的声明末尾时，它表示这个函数是一个常量成员函数。这意味着该函数不能修改对象的任何成员变量（除非这些成员变量被声明为mutable）

 private:
  ReferenceLine reference_line_;
  std::vector<Obstacle> dynamic_obstacles_;
  std::vector<Obstacle> static_obstacles_;
  Config config_;
};
}  // namespace ahrs