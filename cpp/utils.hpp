#include "Eigen/src/Core/Matrix.h"
#include <Eigen/Dense>

// add include guard
#ifndef DOUBLE_INTEGRATOR_HPP
#define DOUBLE_INTEGRATOR_HPP

namespace double_integrator_planning {

struct State {
  // initialize state with 4 doubles
  State(double x0, double x1, double v0, double v1) : x{x0, x1}, v{v0, v1} {}
  State(Eigen::Vector2d x, Eigen::Vector2d v) : x{x}, v{v} {}
  Eigen::Vector2d x;
  Eigen::Vector2d v;

  Eigen::Vector4d to_vector() const {
    Eigen::Vector4d vec;
    vec << x(0), x(1), v(0), v(1);
    return vec;
  }
};

struct BoundingBox {
  Eigen::Vector2d x_min, x_max, v_min, v_max;

  BoundingBox(Eigen::Vector2d x_min, Eigen::Vector2d x_max,
              Eigen::Vector2d v_min, Eigen::Vector2d v_max)
      : x_min{x_min}, x_max{x_max}, v_min{v_min}, v_max{v_max} {};
  BoundingBox(const State &s_min, const State &s_max)
      : x_min{s_min.x}, x_max{s_max.x}, v_min{s_min.v}, v_max{s_max.v} {};

  bool is_inside(const State &s) const {
    return (s.x(0) >= x_min(0) && s.x(0) <= x_max(0) && s.x(1) >= x_min(1) &&
            s.x(1) <= x_max(1) && s.v(0) >= v_min(0) && s.v(0) <= v_max(0) &&
            s.v(1) >= v_min(1) && s.v(1) <= v_max(1));
  };

  State sample() const {
    Eigen::Vector2d x =
        x_min + Eigen::Vector2d::Random().cwiseProduct(x_max - x_min);
    Eigen::Vector2d v =
        v_min + Eigen::Vector2d::Random().cwiseProduct(v_max - v_min);
    return State{x, v};
  };
};

} // namespace double_integrator_planning

#endif
