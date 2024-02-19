#include "double_integrator.hpp"
#include "utils.hpp"
#include <Eigen/Dense>
#include <algorithm>
#include <functional>
#include <memory>
#include <optional>

namespace double_integrator_planning {

struct Node {
  State state;
  std::shared_ptr<Node> parent;
  std::optional<double> duration_from_parent;
  std::optional<double> cost;
};

enum class ExtendResult { REACHED, ADVANCED, TRAPPED };

template <typename NodeT> class _PlannerBase {
public:
  _PlannerBase(const State &start, const State &goal,
               std::function<bool(State)> is_obstacle_free,
               const BoundingBox &state_bound, const double dt_resolution)
      : start_(start), goal_(goal), is_obstacle_free_(is_obstacle_free),
        state_bound_(state_bound), dt_resolution_(dt_resolution) {
    this->nodes_.push_back(
        std::make_shared<NodeT>(NodeT{start, nullptr, {}, {}}));
  }

  bool is_valid(const State &state) const {
    return is_obstacle_free_(state) and state_bound_.is_inside(state);
  }

  Trajectory get_solution() const {
    std::vector<TrajectoryPiece> solution;
    auto node = nodes_.back();
    while (node->parent != nullptr) {
      solution.push_back(TrajectoryPiece(node->parent->state, node->state,
                                         *node->duration_from_parent));
      node = node->parent;
    }
    std::reverse(solution.begin(), solution.end());
    return Trajectory{solution};
  }

  std::vector<TrajectoryPiece> get_all_motions() const {
    std::vector<TrajectoryPiece> motions;
    for (const auto &node : nodes_) {
      if (node->parent != nullptr) {
        motions.push_back(TrajectoryPiece(node->parent->state, node->state,
                                          *node->duration_from_parent));
      }
    }
    return motions;
  }

  virtual bool solve(size_t max_iter) = 0;
  virtual ~_PlannerBase() = default;

public:
  State start_;
  State goal_;
  std::function<bool(State)> is_obstacle_free_;
  BoundingBox state_bound_;
  double dt_resolution_;
  std::vector<std::shared_ptr<NodeT>> nodes_;
};

class RRT : public _PlannerBase<Node> {
public:
  RRT(const State &start, const State &goal,
      std::function<bool(State)> is_obstacle_free,
      const BoundingBox &state_bound, const double dt_extend,
      const double dt_resolution);
  std::shared_ptr<Node> find_nearest(const State &state) const;

  ExtendResult extend(const State &s_target);
  bool solve(size_t max_iter);

public:
  double dt_extend_;
};

enum class FMTNodeStatus { UNVISITED, OPEN, CLOSED };

struct NodeWithStatus {
  State state;
  std::shared_ptr<NodeWithStatus> parent;
  std::optional<double> duration_from_parent;
  std::optional<double> cost;
  FMTNodeStatus status = FMTNodeStatus::OPEN;
};

class FastMarchingTree : public _PlannerBase<NodeWithStatus> {
public:
  FastMarchingTree(const State &start, const State &goal,
                   std::function<bool(State)> is_obstacle_free,
                   const BoundingBox &state_bound, double dt_resolution,
                   double admissible_cost, size_t N);

  bool is_solved();
  bool is_failed();
  size_t get_num_status(FMTNodeStatus status);

  void extend();
  bool solve(size_t max_iter);

  enum class FilterMode { FORWARD, BACKWARD };

  std::tuple<std::vector<std::shared_ptr<NodeWithStatus>>, std::vector<double>,
             std::vector<double>>
  filter_reachable(const State &s_center, double admisible_cost,
                   FilterMode mode, FMTNodeStatus status,
                   bool return_costs) const;

public:
  double addmissible_cost_;
};

} // namespace double_integrator_planning
