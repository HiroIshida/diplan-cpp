#include "double_integrator.hpp"
#include <gtest/gtest.h>

TEST(DoubleIntegrator, OPTIMAL_COST) {
  Eigen::Vector4d s0(0.2, 0.3, 0, 0);
  Eigen::Vector4d s1(1, 1, 0, 0);
  auto [c, t] = optimal_cost(s0, s1);
  EXPECT_NEAR(c, 3.367317015231521, 1e-4);
  EXPECT_NEAR(t, 2.5260490633667847, 1e-4);

}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
