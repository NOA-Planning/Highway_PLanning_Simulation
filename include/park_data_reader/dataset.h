#ifndef DATASET_H
#define DATASET_H

#include <pybind11/embed.h>
#include <pybind11/stl.h>

#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <string>
#include <unordered_map>
#include <vector>

namespace py = pybind11;

struct Frame {
  std::string token;
  double timestamp;
  std::string next;
  std::string prev;
  std::vector<std::string> instances;
};

struct Agent {
  std::string token;
  std::string first_instance;
  std::string type;
  std::vector<double> size;
};

struct Instance {
  std::string token;
  std::vector<double> coords;
  double heading;
  double speed;
  std::string next;
  std::string prev;
  std::string agent_token;
};

struct Obstacle {
  std::string token;
  std::vector<double> coords;
  std::vector<double> size;
  double heading;
};

struct Scene {
  std::string scene_token;
  std::string first_frame;
  std::vector<std::string> obstacles;
};

struct ParkingArea {
  std::vector<Eigen::Vector2d> bounds;
  struct Area {
    std::vector<Eigen::Vector2d> coords;
    Eigen::Vector2i shape;
  };
  std::vector<Area> areas;
};

struct Waypoint {
  std::vector<Eigen::Vector2d> bounds;
  int nums;
};

class Dataset {
 public:
  std::unordered_map<std::string, Frame> frames;
  std::unordered_map<std::string, Agent> agents;
  std::unordered_map<std::string, Instance> instances;
  std::unordered_map<std::string, Obstacle> obstacles;
  std::unordered_map<std::string, Scene> scenes;

  void load_from_python(const std::string& script, const std::string& function,
                        const std::string& filename) {
    py::scoped_interpreter guard{};
    py::module_ sys = py::module_::import("sys");
    sys.attr("path").attr("append")(
        "/home/ahrs/workspace/nday/bspline_lattice_planner/scripts");
    py::module_ dataset_module = py::module_::import(script.c_str());
    py::object get_data = dataset_module.attr(function.c_str());

    std::string data = get_data(filename).cast<std::string>();
    auto json_data = nlohmann::json::parse(data);

    for (const auto& item : json_data["frames"].items()) {
      Frame frame;
      frame.token = item.key();
      frame.timestamp = item.value().at("timestamp").get<double>();
      frame.next = item.value().at("next").get<std::string>();
      frame.prev = item.value().at("prev").get<std::string>();
      frame.instances =
          item.value().at("instances").get<std::vector<std::string>>();
      frames[frame.token] = frame;
    }

    for (const auto& item : json_data["agents"].items()) {
      Agent agent;
      agent.token = item.key();
      agent.first_instance =
          item.value().at("first_instance").get<std::string>();
      agent.type = item.value().at("type").get<std::string>();
      agent.size = item.value().at("size").get<std::vector<double>>();
      agents[agent.token] = agent;
    }

    for (const auto& item : json_data["instances"].items()) {
      Instance instance;
      instance.token = item.key();
      instance.coords = item.value().at("coords").get<std::vector<double>>();
      instance.heading = item.value().at("heading").get<double>();
      instance.speed = item.value().at("speed").get<double>();
      instance.next = item.value().at("next").get<std::string>();
      instance.prev = item.value().at("prev").get<std::string>();
      instance.agent_token = item.value().at("agent_token").get<std::string>();
      instances[instance.token] = instance;
    }

    for (const auto& item : json_data["obstacles"].items()) {
      Obstacle obstacle;
      obstacle.token = item.key();
      obstacle.coords = item.value().at("coords").get<std::vector<double>>();
      obstacle.size = item.value().at("size").get<std::vector<double>>();
      obstacle.heading = item.value().at("heading").get<double>();
      obstacles[obstacle.token] = obstacle;
    }

    for (const auto& item : json_data["scenes"].items()) {
      Scene scene;
      scene.scene_token = item.key();
      scene.first_frame = item.value().at("first_frame").get<std::string>();
      scene.obstacles =
          item.value().at("obstacles").get<std::vector<std::string>>();
      scenes[scene.scene_token] = scene;
    }
  }

  std::vector<std::string> list_scenes() {
    std::vector<std::string> scene_tokens;
    for (const auto& scene : scenes) {
      scene_tokens.push_back(scene.first);
    }
    return scene_tokens;
  }
};

#endif  // DATASET_H
