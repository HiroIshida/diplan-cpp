#include "rrt.hpp"
#include "utils.hpp"
#include <Eigen/Dense>
#include <gtest/gtest.h>

namespace dip = double_integrator_planning;

TEST(RRT, RRT) {
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
  auto rrt = dip::RRT(start, goal, is_obstacle_free, sbound, 0.3, 0.1);

  bool is_solved = rrt.solve(100000);
  EXPECT_TRUE(is_solved);
  auto nodes = rrt.get_solution();
  EXPECT_TRUE(nodes.size() > 0);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
