#include <algorithm>

#include "dp_path.h"
namespace ahrs {
DpPath::DpPath(const std::vector<std::vector<Vec2d>>& sample_points,
               const Environment& env, const Config& config)
    : env_(env), config_(config) {
  //初始化dp组数
  Init(sample_points);
}

std::vector<Vec2d> DpPath::DpSearch() {
  //第0层赋初始 tips:一开始没有赋初值导致计算有问题
  // tips:这里写法很容易崩
  dp_table_.front().front().cost_ = 0;
  // tips:这里是不是应该用二叉树的回溯获取所有路径啊
  for (int i = 1; i < dp_table_.size(); i++) {
    auto& pre_layer = dp_table_[i - 1];
    for (int j = 0; j < dp_table_[i].size(); ++j) {
      double min_cost = std::numeric_limits<double>::max();
      for (auto& parent_point : pre_layer) {
        // cost计算todo
        double cost = static_cast<double>(j);  //模拟一个靠左优先
        cost += parent_point.cost_;
        if (cost < min_cost) {
          Node& current_node = dp_table_[i][j];
          min_cost = cost;
          current_node.parent_index = parent_point.index_;
          current_node.cost_ = min_cost;
        }
      }
    }
  }

  // tips:这里直接访问back，有崩溃的风险
  const auto& final_layer = dp_table_.back();
  double min_cost = std::numeric_limits<double>::max();
  Index min_cost_index(0, 0);
  // tips:这里用std::min_element直接获得，需要写仿函数
  for (const auto& p : final_layer) {
    if (p.cost_ < min_cost) {
      min_cost = p.cost_;
      min_cost_index = p.index_;
    }
  }

  //从最后一层到第零层提取路径
  std::vector<Vec2d> path;
  while (min_cost_index.i_ != -1) {
    const auto& current_node = dp_table_[min_cost_index.i_][min_cost_index.j_];
    path.push_back(current_node.pose_);
    min_cost_index = current_node.parent_index;
  }

  std::reverse(path.begin(), path.end());
  return path;
}

void DpPath::Init(const std::vector<std::vector<Vec2d>>& sample_points) {
  // tips:这里可以用二维指针**代替，否则实例化node效率低，顺带讲一下new和delete的实践，引出智能指针
  for (int i = 0; i < sample_points.size(); ++i) {
    std::vector<Node> layer;  // tips:这里可以提前分配好容量
    for (int j = 0; j < sample_points[i].size(); ++j) {
      layer.emplace_back(Node(sample_points[i][j], Index(i, j)));
    }
    dp_table_.push_back(layer);
  }
}

}  // namespace ahrs
