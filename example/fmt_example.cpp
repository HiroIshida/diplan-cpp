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
  auto v_abs_max = 0.5;
  auto sbound = dip::BoundingBox(dip::State(0, 0, -v_abs_max, -v_abs_max),
                                 dip::State(1.0, 1.0, v_abs_max, v_abs_max));
  auto fmt = dip::FastMarchingTree(start, goal, is_obstacle_free, sbound, 0.1,
                                   1.5, 6000);
  // measure time
  auto start_time = std::chrono::system_clock::now();
  while (true) {
    if (fmt.is_solved()) {
      std::cout << "success" << std::endl;
      break;
    }
    if (fmt.is_failed()) {
      std::cout << "failed" << std::endl;
      break;
    }
    fmt.extend();
  }
  auto end_time = std::chrono::system_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                     end_time - start_time)
                     .count();
  std::cout << "elapsed time: " << elapsed << "[ms]" << std::endl;
  double L = fmt.get_solution().get_length();
  std::cout << "path length: " << L << std::endl;
  fmt.visualize();
  plt::show();
}
