#pragma once
namespace ahrs {
class Config {
 public:
  Config(){};
  double length_ = 40.0;
  double tail_ = 1;
  double front_ = 3;
  double width_ = 2;
  double interval_ = 0.1;  // 10hz
};
}  // namespace ahrs