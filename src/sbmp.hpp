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

class RRT {
public:
  RRT(const State &start, const State &goal,
      std::function<bool(State)> is_obstacle_free,
      const BoundingBox &state_bound, const double dt_extend,
      const double dt_resolution);
  bool is_valid(const State &state) const;
  std::shared_ptr<Node> find_nearest(const State &state) const;

  ExtendResult extend(const State &s_target);
  bool solve(size_t max_iter);
  std::vector<std::shared_ptr<Node>> get_solution() const;

public:
  State start_;
  State goal_;
  // is obstacle free std::function
  std::function<bool(State)> is_obstacle_free_;
  BoundingBox state_bound_;
  double dt_extend_;
  double dt_resolution_;
  std::vector<std::shared_ptr<Node>> nodes_;
};

enum class FMTNodeStatus { UNVISITED, OPEN, CLOSED };

struct NodeWithStatus {
  State state;
  std::shared_ptr<NodeWithStatus> parent;
  std::optional<double> duration_from_parent;
  std::optional<double> cost;
  FMTNodeStatus status;
};

class FastMarchingTree {
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
  bool is_valid(const State &state) const;
  std::vector<TrajectoryPiece> get_solution() const;

public:
  State start_;
  State goal_;
  std::vector<std::shared_ptr<NodeWithStatus>> nodes_;
  std::function<bool(State)> is_obstacle_free_;
  BoundingBox state_bound_;
  double dt_;
  double addmissible_cost_;
};

} // namespace double_integrator_planning
