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

  rrt.visualize();
  plt::show();
}
