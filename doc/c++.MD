1.push_back和emplace_back与结构体共用时

```c++
struct GridNode {
  GridNode(int i = 0, int j = 0) : index_(i, j), pos(0.0, 0.0), valid_(true) {}
  Index index_;
  Vec2d pos_;
  std::vector<Index> neighbors_;
  bool valid_;
};


nodes_.reserve(n * m);
for (int i = 0; i < n_; ++i) {
    for (int j = 0; j < m_; ++j) {
    nodes_.emplace_back(i, j);
    nodes_.push_back(i, j); 
    nodes_.push_back({i,j}); 
    }
}
```

2.一维数组和二维数组的区别
3.指针和实例的区别，存储的位置不同，堆和栈
4.new 二维指针数组的好处
```c++
  std::vector<GridNode> nodes_;//一维和二维都可以
  GridGraph(int n, int m) : n_(n), m_(m) {
    nodes_.reserve(n * m);
    for (int i = 0; i < n_; ++i) {
      for (int j = 0; j < m_; ++j) {
        nodes_.emplace_back(i, j);
      }
    }
  };
```

5.构造函数中成员变量类的构造
```c++
// pos_是一个Vec2d类，这里可以调用Vec2d的构造函数，也可以直接用Vec2d来构造
//默认值需要写在声明里，不能写在函数实现里
  GridNode(int i = 0, int j = 0, const double& x, const double& y)
      : index_(i, j), pos_(x, y), valid_(true) {}
  GridNode(int i = 0, int j = 0, const Vec2d& pos)
      : index_(i, j), pos_(pos), valid_(true) {}
```


6.size() - 1的风险
```c++
for (int x = 0; x < graph_.size() - 1; ++x)
//size_t类型不要-1,容易得到正无穷导致死循环
```

7. reserve和resize的用法和区别
```c++
  std::vector<std::vector<GridNode>> graph_; 
  graph_.reserve(layer_num_);
  graph_.resize(layer_num_);
  //与声明时的区别
  vector<vector<Node>> graph_(m, vector<Node>());


```