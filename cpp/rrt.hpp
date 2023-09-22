#include "double_integrator.hpp"
#include "utils.hpp"
#include <Eigen/Dense>
#include <functional>
#include <memory>
#include <optional>

namespace rrt {

namespace di = double_integrator;

struct Node {
  di::State state;
  std::optional<double> duration_to_parent;
  std::shared_ptr<Node> parent;
};

enum class ExtendResult { REACHED, ADVANCED, TRAPPED };

class RRT {
public:
  RRT(const di::State &start, const di::State &goal,
      std::function<bool(di::State)> is_obstacle_free,
      const double_integrator::BoundingBox &state_bound, const double dt_extend,
      const double dt_resolution)
      : start_(start), goal_(goal), is_obstacle_free_(is_obstacle_free),
        state_bound_(state_bound), dt_extend_(dt_extend),
        dt_resolution_(dt_resolution) {
    this->nodes_.push_back(std::make_shared<Node>(Node{start, {}, nullptr}));
  }

  bool is_valid(const di::State &state) const {
    return is_obstacle_free_(state) and state_bound_.is_inside(state);
  }

  std::shared_ptr<Node> find_nearest(const di::State &state) const {
    std::shared_ptr<Node> nearest_node = nullptr;
    double min_dist = std::numeric_limits<double>::max();
    for (const auto &node : this->nodes_) {
      const double dist = (node->state.to_vector() - state.to_vector()).norm();
      if (dist < min_dist) {
        min_dist = dist;
        nearest_node = node;
      }
    }
    return nearest_node;
  }

  ExtendResult extend(const di::State &s_target) {
    if (!is_valid(s_target)) {
      return ExtendResult::TRAPPED;
    }

    const auto nearest_node = find_nearest(s_target);
    auto [_, t_optimal] = di::optimal_cost(nearest_node->state, s_target);
    auto traj_piece =
        di::TrajectoryPiece(nearest_node->state, s_target, t_optimal);
    double t = dt_resolution_;
    std::optional<di::State> state_to_add = std::nullopt;
    ExtendResult er = ExtendResult::TRAPPED;
    while (t < dt_extend_) {
      auto state = traj_piece.interpolate(t);
      if (not is_valid(state)) {
        if (state_to_add.has_value()) {
          this->nodes_.push_back(
              std::make_shared<Node>(Node{*state_to_add, t, nearest_node}));
        }
        return er;
      } else {
        state_to_add = std::move(state);
        er = ExtendResult::ADVANCED;
      }
      t += dt_resolution_;
    }
    if (state_to_add.has_value()) {
      this->nodes_.push_back(std::make_shared<Node>(
          Node{*state_to_add, dt_extend_, nearest_node}));
    }
    return ExtendResult::REACHED;
  }

  bool solve(size_t max_iter) {
    for (size_t i = 0; i < max_iter; ++i) {
      const auto s_rand = state_bound_.sample();
      const auto extend_result = extend(s_rand);
      if (extend_result != ExtendResult::TRAPPED) {
        auto [time_optimal, _] = di::optimal_cost(nodes_.back()->state, goal_);
        if (time_optimal < dt_extend_) {
          auto traj_piece =
              di::TrajectoryPiece(nodes_.back()->state, goal_, time_optimal);

          auto is_connectable = [&]() {
            double t = dt_resolution_;
            while (t < time_optimal) {
              auto state = traj_piece.interpolate(t);
              if (not is_valid(state)) {
                return false;
              }
              t += dt_resolution_;
            }
            return true;
          };

          if (is_connectable()) {
            nodes_.push_back(std::make_shared<Node>(
                Node{goal_, time_optimal, nodes_.back()}));
            return true;
          }
        }
      }
    }
    return false;
  }

  std::vector<std::shared_ptr<Node>> get_solution() const {
    std::vector<std::shared_ptr<Node>> solution;
    auto node = nodes_.back();
    while (node != nullptr) {
      solution.push_back(node);
      node = node->parent;
    }
    std::reverse(solution.begin(), solution.end());
    return solution;
  }

public:
  di::State start_;
  di::State goal_;
  // is obstacle free std::function
  std::function<bool(di::State)> is_obstacle_free_;
  double_integrator::BoundingBox state_bound_;
  double dt_extend_;
  double dt_resolution_;
  std::vector<std::shared_ptr<Node>> nodes_;
};

} // namespace rrt
