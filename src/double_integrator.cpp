#include "double_integrator.hpp"

namespace double_integrator_planning {

double bisection_newton(const std::function<double(double)> &f,
                        const std::function<double(double)> &df, double start,
                        double end, double tol, int max_iter) {
  double est = (start + end) * 0.5;
  for (int i = 0; i < max_iter; ++i) {
    double fratio = f(est) / df(est);

    if ((end - (est - fratio)) * ((est - fratio) - start) < 0.0 ||
        std::abs(fratio) < (end - start) / 4.0) {
      if (f(est) > 0) {
        end = est;
      } else {
        start = est;
      }
      fratio = est - (end + start) * 0.5;
    }
    est -= fratio;

    if (std::abs(fratio) < tol) {
      break;
    }
  }
  return est;
}

std::pair<double, double> optimal_cost(const State &s0, const State &s1) {
  const Eigen::Vector2d &x0 = s0.x;
  const Eigen::Vector2d &v0 = s0.v;
  const Eigen::Vector2d &x1 = s1.x;
  const Eigen::Vector2d &v1 = s1.v;
  Eigen::Vector2d x_diff = x1 - x0;
  Eigen::Vector2d v_diff = v1 - v0;

  double p = -4 * (v0.dot(v0) + v1.dot(v1) + v0.dot(v1));
  double q = 24 * (v0 + v1).dot(x_diff);
  double r = -36 * x_diff.dot(x_diff);

  auto cost = [&](double t) {
    return t + v_diff.dot(4.0 * v_diff / t - 6 * (-v0 * t + x_diff) / (t * t)) +
           (-6 * v_diff / (t * t) + 12 * (x_diff - v0 * t) / (t * t * t))
               .dot(-v0 * t + x_diff);
  };

  auto cost_d1 = [&](double t) {
    return t * t * t * t + p * t * t + q * t + r;
  };

  auto cost_d2 = [&](double t) { return 4.0 * t * t * t + 2 * p * t + q; };

  double t_min = 0.0;
  double t_max = 10.0;
  double optimal_t = bisection_newton(cost_d1, cost_d2, t_min, t_max);
  return {cost(optimal_t), optimal_t};
}

BoundingBox forward_reachable_box(const State &state, double r) {
  Eigen::Vector2d tau_x_plus, tau_x_minus, xmax, xmin, vmax, vmin;
  const Eigen::Vector2d &x0 = state.x;
  const Eigen::Vector2d &v0 = state.v;

  tau_x_plus = 2.0 / 3.0 *
               (-v0.array().square() + r +
                (v0.array() * (v0.array().square() + r).sqrt()));
  tau_x_minus = 2.0 / 3.0 *
                (-v0.array().square() + r -
                 (v0.array() * (v0.array().square() + r).sqrt()));

  xmax = (v0.cwiseProduct(tau_x_plus) + x0).array() +
         ((1.0 / 3.0) * tau_x_plus.array().square() * (-tau_x_plus.array() + r))
             .sqrt();
  xmin =
      (v0.cwiseProduct(tau_x_minus) + x0).array() -
      ((1.0 / 3.0) * tau_x_minus.array().square() * (-tau_x_minus.array() + r))
          .sqrt();

  double tau_v_plus = 0.5 * r;
  vmax =
      v0 + Eigen::Vector2d::Constant(std::sqrt(tau_v_plus * (-tau_v_plus + r)));
  vmin =
      v0 - Eigen::Vector2d::Constant(std::sqrt(tau_v_plus * (-tau_v_plus + r)));
  return BoundingBox{xmin, xmax, vmin, vmax};
}

BoundingBox backward_reachable_box(const State &state, double r) {
  Eigen::Vector2d tau_x_plus, tau_x_minus, xmax, xmin, vmax, vmin;
  const Eigen::Vector2d &x0 = state.x;
  const Eigen::Vector2d &v0 = state.v;

  tau_x_plus = 2.0 / 3.0 *
               (v0.array().square() - r +
                (v0.array() * (v0.array().square() + r).sqrt()));
  tau_x_minus = 2.0 / 3.0 *
                (v0.array().square() - r -
                 (v0.array() * (v0.array().square() + r).sqrt()));

  xmax = (v0.cwiseProduct(tau_x_plus) + x0).array() +
         ((1.0 / 3.0) * tau_x_plus.array().square() * (tau_x_plus.array() + r))
             .sqrt();
  xmin =
      (v0.cwiseProduct(tau_x_minus) + x0).array() -
      ((1.0 / 3.0) * tau_x_minus.array().square() * (tau_x_minus.array() + r))
          .sqrt();

  double tau_v_plus = 0.5 * r;
  vmax =
      v0 + Eigen::Vector2d::Constant(std::sqrt(tau_v_plus * (tau_v_plus + r)));
  vmin =
      v0 - Eigen::Vector2d::Constant(std::sqrt(tau_v_plus * (tau_v_plus + r)));
  return BoundingBox{xmin, xmax, vmin, vmax};
}

TrajectoryPiece::TrajectoryPiece(const State &s0, const State &s1,
                                 double duration)
    : duration(duration), s1_vec(s1.to_vector()) {
  Eigen::Vector2d x0 = s0.x;
  Eigen::Vector2d v0 = s0.v;
  Eigen::Vector2d x1 = s1.x;
  Eigen::Vector2d v1 = s1.v;

  Eigen::Vector2d x01 = x1 - x0;
  Eigen::Vector2d v01 = v1 - v0;

  d.head<2>() =
      -6.0 * v01 / (duration * duration) +
      12.0 * (-duration * v0 + x01) / (duration * duration * duration);
  d.tail<2>() = 4.0 * v01 / duration -
                6.0 * (-duration * v0 + x01) / (duration * duration);
}

State TrajectoryPiece::interpolate(double t) const {
  double s = t - duration;
  Eigen::Matrix4d M_left = Eigen::Matrix4d::Zero();
  Eigen::Matrix4d M_right = Eigen::Matrix4d::Zero();

  M_left.block<2, 2>(0, 0) = Eigen::Matrix2d::Identity();
  M_left.block<2, 2>(0, 2) = Eigen::Matrix2d::Identity() * s;
  M_left.block<2, 2>(2, 2) = Eigen::Matrix2d::Identity();

  M_right.block<2, 2>(0, 0) = Eigen::Matrix2d::Identity() * (-s * s * s / 6.0);
  M_right.block<2, 2>(0, 2) = Eigen::Matrix2d::Identity() * s * s * 0.5;
  M_right.block<2, 2>(2, 0) = Eigen::Matrix2d::Identity() * (-s * s * 0.5);
  M_right.block<2, 2>(2, 2) = Eigen::Matrix2d::Identity() * s;

  auto vec = M_left * s1_vec + M_right * d;
  return State{vec.head<2>(), vec.tail<2>()};
}

double Trajectory::get_duration() const {
  double duration = 0;
  for (const auto &piece : pieces) {
    duration += piece.duration;
  }
  return duration;
}

State Trajectory::interpolate(double t) const {
  if (t < 0 || t > get_duration() + 1e-6) {
    throw std::runtime_error("Trajectory::interpolate: t out of bounds");
  }
  double t_left = t;
  for (const auto &piece : pieces) {
    if (t_left < piece.duration) {
      return piece.interpolate(t_left);
    }
    t_left -= piece.duration;
  }
  return pieces.back().interpolate(pieces.back().duration);
}

} // namespace double_integrator_planning
