#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <tuple>
#include <vector>

#include "utils.hpp"

namespace double_integrator_planning {

double bisection_newton(const std::function<double(double)> &f,
                        const std::function<double(double)> &df, double start,
                        double end, double tol = 0.05, int max_iter = 20);

std::pair<double, double> optimal_cost(const State &s0, const State &s1);

BoundingBox forward_reachable_box(const State &state, double r);

BoundingBox backward_reachable_box(const State &state, double r);

class TrajectoryPiece {
public:
  double duration_;

private:
  Eigen::Vector4d _d;
  Eigen::Vector4d _s1;

public:
  TrajectoryPiece(const State &s0, const State &s1, double duration);
  State interpolate(double t) const;
};

} // namespace double_integrator_planning
