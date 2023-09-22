#include "double_integrator.hpp"
#include "utils.hpp"
#include <Eigen/Dense>
#include <algorithm>
#include <functional>
#include <memory>
#include <optional>

namespace rrt {

namespace di = double_integrator;

struct Node {
  di::State state;
  std::shared_ptr<Node> parent;
  std::optional<double> duration_from_parent;
  std::optional<double> cost;
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
    this->nodes_.push_back(
        std::make_shared<Node>(Node{start, nullptr, {}, {}}));
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
              std::make_shared<Node>(Node{*state_to_add, nearest_node, t, {}}));
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
          Node{*state_to_add, nearest_node, dt_extend_, {}}));
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
                Node{goal_, nodes_.back(), time_optimal}));
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

enum class FMTNodeStatus { UNVISITED, OPEN, CLOSED };

struct NodeWithStatus {
  di::State state;
  std::shared_ptr<NodeWithStatus> parent;
  std::optional<double> duration_from_parent;
  std::optional<double> cost;
  FMTNodeStatus status;
};

class FastMarchingTree {
public:
  FastMarchingTree(const di::State &start, const di::State &goal,
                   std::function<bool(di::State)> is_obstacle_free,
                   const double_integrator::BoundingBox &state_bound,
                   double dt_resolution, double admissible_cost, size_t N)
      : start_(start), goal_(goal), is_obstacle_free_(is_obstacle_free),
        state_bound_(state_bound), dt_(dt_resolution),
        addmissible_cost_(admissible_cost) {
    nodes_.push_back(std::make_shared<NodeWithStatus>(
        NodeWithStatus{start_, nullptr, {}, 0.0, FMTNodeStatus::OPEN}));
    // sample N state
    for (size_t i = 0; i < N; ++i) {
      while (true) {
        auto s_rand = state_bound_.sample();
        if (is_valid(s_rand)) {
          nodes_.push_back(std::make_shared<NodeWithStatus>(NodeWithStatus{
              s_rand, nullptr, {}, {}, FMTNodeStatus::UNVISITED}));
          break;
        }
      }
    }
    nodes_.push_back(std::make_shared<NodeWithStatus>(
        NodeWithStatus{goal_, nullptr, {}, {}, FMTNodeStatus::UNVISITED}));
  }

  bool is_solved() {
    auto &goal_node = nodes_.back();
    return goal_node->status != FMTNodeStatus::UNVISITED;
  }
  bool is_failed() { return get_num_status(FMTNodeStatus::OPEN) == 0; }

  size_t get_num_status(FMTNodeStatus status) {
    size_t num = 0;
    for (const auto &node : nodes_) {
      if (node->status == status) {
        num++;
      }
    }
    return num;
  }

  void extend() {
    std::shared_ptr<NodeWithStatus> open_node_best = nullptr;
    double open_node_cost_min = std::numeric_limits<double>::max();
    for (const auto &node : nodes_) {
      if (node->status == FMTNodeStatus::OPEN) {
        if (*node->cost < open_node_cost_min) {
          open_node_best = node;
          open_node_cost_min = *node->cost;
        }
      }
    }
    assert(open_node_best->status == FMTNodeStatus::OPEN);

    auto [nodes_reachable, _, __] =
        filter_reachable(open_node_best->state, addmissible_cost_,
                         FilterMode::FORWARD, FMTNodeStatus::UNVISITED, false);
    for (const auto &node : nodes_reachable) {
      assert(node->status == FMTNodeStatus::UNVISITED);
      auto [nodes_cand, costs_cand, times_cand] =
          filter_reachable(node->state, addmissible_cost_, FilterMode::BACKWARD,
                           FMTNodeStatus::OPEN, true);
      if (nodes_cand.empty())
        continue;

      double cost_optimal = std::numeric_limits<double>::max();
      size_t i_optimal = 0;
      for (size_t i = 0; i < nodes_cand.size(); ++i) {
        double cost = costs_cand[i] + *nodes_cand[i]->cost;
        if (cost < cost_optimal) {
          cost_optimal = cost;
          i_optimal = i;
        }
      }
      std::shared_ptr<NodeWithStatus> parent_optimal = nodes_cand[i_optimal];
      double time_optimal = times_cand[i_optimal];
      auto is_connectable = [&]() {
        auto traj = di::TrajectoryPiece(parent_optimal->state, node->state,
                                        time_optimal);
        for (double t = dt_; t < time_optimal; t += dt_) {
          if (!is_valid(traj.interpolate(t))) {
            return false;
          }
        }
        return true;
      };
      if (is_connectable()) {
        node->parent = parent_optimal;
        node->cost = *parent_optimal->cost + cost_optimal;
        node->status = FMTNodeStatus::OPEN;
        node->duration_from_parent = time_optimal;
      }
    }
    open_node_best->status = FMTNodeStatus::CLOSED;
  }

  enum class FilterMode { FORWARD, BACKWARD };

  std::tuple<std::vector<std::shared_ptr<NodeWithStatus>>, std::vector<double>,
             std::vector<double>>
  filter_reachable(const di::State &s_center, double admisible_cost,
                   FilterMode mode, FMTNodeStatus status,
                   bool return_costs) const {
    std::vector<std::shared_ptr<NodeWithStatus>> node_reachable;
    std::vector<double> costs_reachable;
    std::vector<double> times_reachable;

    auto box = (mode == FilterMode::FORWARD)
                   ? di::forward_reachable_box(s_center, admisible_cost)
                   : di::backward_reachable_box(s_center, admisible_cost);
    for (const auto &node : nodes_) {
      if (node->status != status)
        continue;
      if (box.is_inside(node->state)) {
        auto [c, t] = (mode == FilterMode::FORWARD)
                          ? di::optimal_cost(s_center, node->state)
                          : di::optimal_cost(node->state, s_center);
        if (c < admisible_cost) {
          node_reachable.push_back(node);
          if (return_costs) {
            costs_reachable.push_back(c);
            times_reachable.push_back(t);
          }
        }
      }
    }
    return {node_reachable, costs_reachable, times_reachable};
  }

  bool is_valid(const di::State &state) const {
    return is_obstacle_free_(state) and state_bound_.is_inside(state);
  }

  std::vector<std::shared_ptr<NodeWithStatus>> get_solution() const {
    std::vector<std::shared_ptr<NodeWithStatus>> solution;
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
  std::vector<std::shared_ptr<NodeWithStatus>> nodes_;
  std::function<bool(di::State)> is_obstacle_free_;
  double_integrator::BoundingBox state_bound_;
  double addmissible_cost_;
  double dt_;
};

} // namespace rrt
