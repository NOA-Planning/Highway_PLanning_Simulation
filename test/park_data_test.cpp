#include <pybind11/embed.h>
#include <pybind11/stl.h>

#include <iostream>
#include <nlohmann/json.hpp>

#include "park_data_reader/dataset.h"
namespace py = pybind11;

int main() {
  Dataset dataset;
  dataset.load_from_python(
      "dataset", "get_dataset_data",
      "/home/ahrs/workspace/nday/bspline_lattice_planner/data/"
      "DJI_0002");

  // 示例: 打印一些加载的数据
  for (const auto& [token, frame] : dataset.frames) {
    std::cout << "Frame token: " << token << ", timestamp: " << frame.timestamp
              << std::endl;
  }

  return 0;
}
