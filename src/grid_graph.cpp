
#include "config.h"
#include "environment.h"
#include "grid_graph.h"
namespace ahrs {
GridGraph::GridGraph(const std::vector<std::vector<Vec2d>>& sample_points,
                     const Environment& env, const Config& config) {
  layer_num_ = sample_points.size();
  // graph_.reserve(layer_num_);  //
  graph_.resize(layer_num_);

  //创建节点
  for (int x = 0; x < sample_points.size(); ++x) {
    for (int y = 0; y < sample_points[x].size(); ++y) {
      graph_[x].emplace_back(GridNode(x, y, sample_points[x][y]));
    }
  }
  //设置邻接节点
  for (int x = 0; x < graph_.size() - 1; ++x) {
    for (int y = 0; y < graph_[x].size(); ++y) {
      for (int ny = 0; ny < graph_[x + 1].size(); ++ny) {
        graph_[x][y].neighbors_.push_back(&graph_[x + 1][ny]);
      }
    }
  }
}
std::vector<std::vector<Vec2d>> GridGraph::GraphSearch() {
  std::vector<std::vector<Vec2d>> all_paths;
  std::vector<Vec2d> path;
  Dfs(&graph_[0][0], layer_num_ - 1, path, all_paths);
  return all_paths;
}

void GridGraph::Dfs(GridNode* current, int end_x, std::vector<Vec2d>& path,
                    std::vector<std::vector<Vec2d>>& all_paths) {
  if (!current || current->visited_ == true) {
    return;
  }
  current->visited_ = true;
  path.push_back(current->pos_);
  if (current->index_.i_ == end_x) {
    all_paths.push_back(path);
  } else {
    for (GridNode* neighbor : current->neighbors_) {
      Dfs(neighbor, end_x, path, all_paths);
    }
  }

  path.pop_back();
  current->visited_ = false;
}

};  // namespace ahrs