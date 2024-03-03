#pragma once
#include "common.h"
#include "config.h"
#include "environment.h"
namespace ahrs {
struct GridNode {
  GridNode(int i, int j) : index_(i, j), pos_(0.0, 0.0), valid_(true) {}
  GridNode(int i, int j, const double& x, const double& y)
      : index_(i, j), pos_(x, y), valid_(true), visited_(false) {}
  GridNode(int i, int j, const Vec2d& pos)
      : index_(i, j), pos_(pos), valid_(true), visited_(false) {}
  Index index_;
  Vec2d pos_;
  std::vector<GridNode*> neighbors_;
  bool valid_;
  bool visited_;
};

//其实这个graph可以离线生成好啊，只需要将pos赋值给graph就行了
struct GridGraph {
 public:
  int n_, m_;
  int layer_num_;
  std::vector<std::vector<GridNode>> graph_;  //一维和二维都可以
  GridGraph(const std::vector<std::vector<Vec2d>>& sample_points,
            const Environment& env, const Config& config);
  std::vector<std::vector<Vec2d>> GraphSearch();

 private:
  void Dfs(GridNode* current, int end_x, std::vector<Vec2d>& path,
           std::vector<std::vector<Vec2d>>& all_paths);
};

};  // namespace ahrs
