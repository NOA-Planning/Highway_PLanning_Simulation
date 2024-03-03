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


// reserve和resize后的push_back
  for (size_t i = 0; i < points.size(); ++i) {
    std::vector<Vec2d> temp;
    for (const auto& p : points) {
      temp.push_back(p.pos_);
    }
    ShowPoints(temp, name, color, config);
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

8.自定义比较函数的排序

```c++
struct CostPath {
  double cost_;
  std::vector<Node2d> path_;

  //确保了容器中的元素是按照cost_值的升序排列
  // 方法1: 嵌套结构作为比较器
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
struct CompareCost {
  bool operator()(const CostPath& lhs, const CostPath& rhs) const {
    return lhs.cost_ < rhs.cost_;
  }
};
```

在这两种情况中，CostPath::CompareCost的使用略有不同，主要因为std::sort和std::priority_queue的参数期望的形式不同。

对于std::sort
CostPath::CompareCost()使用了圆括号，这意味着你正在创建CostPath::CompareCost类型的一个匿名（临时）对象。std::sort函数需要一个可以调用的比较实体作为其第三个参数，这里使用了一个临时的比较对象实例来满足这个需求。圆括号在这里是构造函数的调用，表示创建了一个比较器对象的实例。

对于std::priority_queue
在这个声明中，CostPath::CompareCost后面没有加圆括号。这是因为在std::priority_queue的模板参数列表中，你需要提供一个类型（而不是对象），这个类型指定了队列如何比较其元素。std::priority_queue使用这个类型内部实例化一个比较器对象，用于其内部元素的排序逻辑。因此，这里你提供的是一个用于比较的类或结构体类型，而不是一个对象实例。

```c++
void BsplineLatticePlanner::SampleScore(
    const std::vector<std::vector<Node2d>>& ctp_seq, const Environment& env,
    const Config& config) {
  std::vector<CostPath> sort_vec;
  TrajectoryScorer scorer;
  double cost = 0.0;
  for (auto& seq : ctp_seq) {
    cost = scorer.GetScorer(seq, env, config);
    sort_vec.emplace_back(CostPath(cost, seq));
  }
  // 1.用CostPath::CompareCost()
  std::sort(sort_vec.begin(), sort_vec.end(), CostPath::CompareCost());
  std::priority_queue<CostPath, std::vector<CostPath>, CostPath::CompareCost>
      pq_;

  // 2.用的是CostPath中的操作符<比较
  std::sort(sort_vec.begin(), sort_vec.end());
  std::set<CostPath> sort_set;
}
```

9.类的默认可见性
实例了一个TrajectoryScorer,发现报错，原来是忘了写public，类的默认可见性是private，而struct的默认可见性是public。

```c++
class TrajectoryScorer {
  TrajectoryScorer() {}
  double GetScorer(std::vector<Node2d>& path, const Environment& env,
                   const Config& config);
};

```
