#pragma once
#include "common.h"
#include "config.h"
#include "environment.h"

namespace ahrs {

struct CostPath {
  CostPath(const double& cost, const std::vector<Node2d>& path)
      : cost_(cost), path_(path) {}
  CostPath() {}
  double cost_;
  std::vector<Node2d> path_;

  //确保了容器中的元素是按照cost_值的升序排列
  // 方法1: 嵌套结构作为比较器（可用于set,vector,priority_queue）
  struct CompareCost {
    bool operator()(const CostPath& lhs, const CostPath& rhs) const {
      return lhs.cost_ < rhs.cost_;
    }
  };
  // 方法2:
  // 在类内直接定义比较操作符（不适用于使用优先级队列时，因为优先级队列需要显示的指明比较函数）
  bool operator<(const CostPath& rhs) const { return cost_ < rhs.cost_; }
};

//方法3: 定义一个结构体
// struct CompareCost {
//   bool operator()(const CostPath& lhs, const CostPath& rhs) const {
//     return lhs.cost_ < rhs.cost_;
//   }
// };

class TrajectoryScorer {
 public:
  TrajectoryScorer() {}
  double GetScorer(const std::vector<Node2d>& path, const Environment& env,
                   const Config& config);
  double SmoothTerm(const std::vector<Node2d>& path, const Environment& env,
                    const Config& config);
  double SafeTerm(const std::vector<Node2d>& path, const Environment& env,
                  const Config& config);
  double TrafficTerm(const std::vector<Node2d>& path, const Environment& env,
                     const Config& config);
  double CurvatureTerm(const std::vector<Node2d>& path, const Environment& env,
                       const Config& config);
  double ConsistencyTerm(const std::vector<Node2d>& path,
                         const Environment& env, const Config& config);
};
}  // namespace ahrs