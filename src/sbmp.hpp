#include "double_integrator.hpp"
#include "utils.hpp"
#include <Eigen/Dense>
#include <algorithm>
#include <functional>
#include <memory>
#include <optional>

#ifdef COMPILE_WITH_MATPLOTLIB
#include "matplotlibcpp.h"
#endif

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

  void visualize() const {
#ifdef COMPILE_WITH_MATPLOTLIB

    // plot nodes
    {
      std::vector<double> xs;
      std::vector<double> ys;
      for (const auto &node : nodes_) {
        xs.push_back(node->state.x(0));
        ys.push_back(node->state.x(1));
      }
      matplotlibcpp::scatter(xs, ys);
    }

    {
      for (const auto &node : nodes_) {
        if (node->parent != nullptr) {
          std::vector<double> piece_xs;
          std::vector<double> piece_ys;

          auto traj = TrajectoryPiece(node->parent->state, node->state,
                                      *node->duration_from_parent);
          for (double t = 0.0; t < *node->duration_from_parent;
               t += dt_resolution_) {
            auto state = traj.interpolate(t);
            piece_xs.push_back(state.x(0));
            piece_ys.push_back(state.x(1));
          }
          piece_xs.push_back(node->state.x(0));
          piece_ys.push_back(node->state.x(1));

          std::map<std::string, std::string> keywords;
          keywords["color"] = "k";
          keywords["linewidth"] = "0.1";
          matplotlibcpp::plot(piece_xs, piece_ys, keywords);
        }
      }
    }

    {
      auto solution = get_solution();
      std::vector<double> sol_xs;
      std::vector<double> sol_ys;
      for (const auto &piece : solution.pieces) {
        for (double t = 0.0; t < piece.duration; t += dt_resolution_) {
          auto state = piece.interpolate(t);
          sol_xs.push_back(state.x(0));
          sol_ys.push_back(state.x(1));
        }
        auto end_state = piece.interpolate(piece.duration);
        sol_xs.push_back(end_state.x(0));
        sol_ys.push_back(end_state.x(1));
      }
      std::map<std::string, std::string> keywords;
      keywords["color"] = "g";
      keywords["linewidth"] = "2.0";
      matplotlibcpp::plot(sol_xs, sol_ys, keywords);
    }

    // start and goal nodes by red
    std::vector<double> xs_tip = {start_.x(0), goal_.x(0)};
    std::vector<double> ys_tip = {start_.x(1), goal_.x(1)};
    std::map<std::string, std::string> keywords;
    keywords["color"] = "red";
    matplotlibcpp::scatter(xs_tip, ys_tip, 30, keywords);
#else
    std::cout << "Matplotlib is not available. Please compile with "
                 "COMPILE_WITH_MATPLOTLIB=ON"
              << std::endl;
#endif
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
