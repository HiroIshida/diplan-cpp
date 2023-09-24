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

struct TrajectoryPiece {
  double duration;
  Eigen::Vector4d d;
  Eigen::Vector4d s1_vec;
  TrajectoryPiece(double duration, const Eigen::Vector4d &d,
                  const Eigen::Vector4d &s1_vec)
      : duration(duration), d(d), s1_vec(s1_vec) {}
  TrajectoryPiece(const State &s0, const State &s1, double duration);
  State interpolate(double t) const;
};

struct Trajectory {
  State get_start() const { return pieces.front().interpolate(0); }
  State get_end() const {
    return pieces.back().interpolate(pieces.back().duration);
  }
  double get_duration() const;
  State interpolate(double t) const;
  std::vector<TrajectoryPiece> pieces;
};

} // namespace double_integrator_planning
