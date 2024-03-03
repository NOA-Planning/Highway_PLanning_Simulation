#pragma once
namespace ahrs {
class Config {
 public:
  Config(){};
  //车辆参数配置
  double tail_ = 1;
  double front_ = 3;
  double width_ = 2;

  //规划器参数配置
  double interval_ = 0.1;  // 10hz

  // 采样参数配置
  double sample_length_ = 20.0;
  double sample_half_width_ = 2.5;
  double ctp_interval_x_ = 5.0;
  double ctp_interval_y_ = 1;
  double zero_layer_interval_ = 0.5 * ctp_interval_x_;
};
}  // namespace ahrs