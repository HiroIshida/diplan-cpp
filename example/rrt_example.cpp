#include "matplotlibcpp.h"
#include "sbmp.hpp"
#include "utils.hpp"
#include <Eigen/Dense>
#include <chrono>
#include <vector>

namespace dip = double_integrator_planning;
namespace plt = matplotlibcpp;

int main() {
  auto is_obstacle_free([](const dip::State &state) -> bool {
    // circle obstacle at (0.5, 0.5) with radius 0.1
    if ((state.x - Eigen::Vector2d(0.5, 0.5)).norm() < 0.4) {
      return false;
    }
    return true;
  });

  auto start = dip::State(0.1, 0.1, 0, 0);
  auto goal = dip::State(0.9, 0.9, 0, 0);
  auto sbound = dip::BoundingBox(dip::State(0, 0, -0.3, -0.3),
                                 dip::State(1.0, 1.0, 0.3, 0.3));
  auto rrt = dip::RRT(start, goal, is_obstacle_free, sbound, 1.0, 0.1);

  // measure solving time in microseconds
  auto start_time = std::chrono::system_clock::now();
  bool is_solved = rrt.solve(100000);
  auto end_time = std::chrono::system_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
                     end_time - start_time)
                     .count();
  std::cout << "elapsed time: " << elapsed << "[mu-s]" << std::endl;

  // all sample xs
  {
    std::vector<double> xs;
    std::vector<double> ys;
    for (const auto &node : rrt.nodes_) {
      xs.push_back(node->state.x(0));
      ys.push_back(node->state.x(1));
    }
    plt::scatter(xs, ys, 10.0, {{"color", "r"}});
  }
  // all edges
  {
    for (const auto &node : rrt.nodes_) {
      if (node->parent) {
        std::vector<double> xs;
        std::vector<double> ys;
        auto parent = node->parent;
        auto traj = dip::TrajectoryPiece(parent->state, node->state,
                                         *node->duration_from_parent);
        for (double t = 0.0; t < *node->duration_from_parent;
             t += rrt.dt_resolution_) {
          auto state = traj.interpolate(t);
          xs.push_back(state.x(0));
          ys.push_back(state.x(1));
        }
        xs.push_back(node->state.x(0));
        ys.push_back(node->state.x(1));
        plt::plot(xs, ys, "r");
      }
    }
  }

  auto sol = rrt.get_solution();
  // all xs in path
  {
    std::vector<double> xs;
    std::vector<double> ys;
    for (const auto &node : sol) {
      xs.push_back(node->state.x(0));
      ys.push_back(node->state.x(1));
    }
    plt::scatter(xs, ys, 20.0, {{"color", "k"}});
  }
  // all edges in path
  {
    auto sol = rrt.get_solution();
    for (const auto &node : sol) {
      if (node->parent) {
        std::vector<double> xs;
        std::vector<double> ys;
        auto parent = node->parent;
        auto traj = dip::TrajectoryPiece(parent->state, node->state,
                                         *node->duration_from_parent);
        for (double t = 0.0; t < *node->duration_from_parent;
             t += rrt.dt_resolution_) {
          auto state = traj.interpolate(t);
          xs.push_back(state.x(0));
          ys.push_back(state.x(1));
        }
        xs.push_back(node->state.x(0));
        ys.push_back(node->state.x(1));
        plt::plot(xs, ys, "k");
      }
    }
  }

  plt::show();
}
