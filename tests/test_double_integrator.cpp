#include "double_integrator.hpp"
#include <gtest/gtest.h>

namespace dip = double_integrator_planning;

TEST(DoubleIntegrator, OPTIMAL_COST) {
  auto s0 = dip::State(0.2, 0.3, 0, 0);
  auto s1 = dip::State(1, 1, 0, 0);

  auto [c, t] = optimal_cost(s0, s1);
  EXPECT_NEAR(c, 3.367317015231521, 1e-4);
  EXPECT_NEAR(t, 2.5260490633667847, 1e-4);
}

TEST(DoubleIntegrator, FORWARD_REACHABLE_BOX) {
  auto s0 = dip::State(0.2, 0.3, 0, 0);
  auto s1 = dip::State(1, 1, 0, 0);
  auto box = dip::forward_reachable_box(s0, 1.0);

  ASSERT_TRUE(box.x_min.isApprox(
      Eigen::Vector2d(-0.022222222222222227, 0.07777777777777775), 1e-4));
  ASSERT_TRUE(box.x_max.isApprox(
      Eigen::Vector2d(0.4222222222222223, 0.5222222222222223), 1e-4));
  ASSERT_TRUE(box.v_min.isApprox(Eigen::Vector2d(-0.5, -0.5), 1e-4));
  ASSERT_TRUE(box.v_max.isApprox(Eigen::Vector2d(0.5, 0.5), 1e-4));
}

TEST(DoubleIntegrator, BACKWARD_REACHABLE_BOX) {
  auto s0 = dip::State(0.2, 0.3, 0, 0);
  auto s1 = dip::State(1, 1, 0, 0);
  auto box = dip::backward_reachable_box(s1, 1.0);
  // ASSERT_TRUE(box.x_min.isApprox(Eigen::Vector2d(-0.022222222222222227,
  // 0.07777777777777775), 1e-4));
  // ASSERT_TRUE(box.x_max.isApprox(Eigen::Vector2d(0.4222222222222223,
  // 0.5222222222222223), 1e-4));
  ASSERT_TRUE(box.v_min.isApprox(
      Eigen::Vector2d(-0.8660254037844386, -0.8660254037844386), 1e-4));
  ASSERT_TRUE(box.v_max.isApprox(
      Eigen::Vector2d(0.8660254037844386, 0.8660254037844386), 1e-4));
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
