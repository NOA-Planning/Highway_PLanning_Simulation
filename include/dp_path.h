#pragma once
#include "common.h"
#include "config.h"
#include "environment.h"
namespace ahrs {

struct Node {
  Node(const Vec2d& sample_point, const Index& index,
       const double& cost = std::numeric_limits<double>::max())
      : pose_(sample_point),
        index_(index),
        parent_index({-1, -1}),
        cost_(cost),
        path_({}) {}
  Vec2d pose_;
  Index index_;
  Index parent_index;
  std::vector<Index> parent_indexes_;
  //   Node* pre_node_; //tips:这里用node*是不是更高效
  double cost_;
  std::vector<Vec2d> path_;
};

class DpPath {
 public:
  DpPath(const std::vector<std::vector<Vec2d>>& sample_points,
         const Environment& env, const Config& config);
  std::vector<Vec2d> DpSearch();

 private:
  void Init(const std::vector<std::vector<Vec2d>>& sample_points);
  std::vector<std::vector<Node>> dp_table_;
  Environment env_;
  Config
      config_;  // tips:很多类都用到了config,没必要每个类都拷贝一份config，可以用单例。
};

}  // namespace ahrs

class DpPath {};