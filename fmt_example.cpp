#include <Eigen/Dense>
#include <vector>
#include <chrono>
#include "Eigen/src/Core/Matrix.h"
#include "utils.hpp"
#include "rrt.hpp"
#include "matplotlibcpp.h"

namespace di = double_integrator;
namespace plt = matplotlibcpp;

int main(){
    auto is_obstacle_free([](const di::State &state) -> bool {
        // circle obstacle at (0.5, 0.5) with radius 0.1
        if ((state.x - Eigen::Vector2d(0.5, 0.5)).norm() < 0.4) {
            return false;
        }
        return true;
    });

    auto start = di::State(0.1, 0.1, 0, 0);
    auto goal = di::State(0.9, 0.9, 0, 0);
    auto sbound = di::BoundingBox(di::State(0, 0, -0.3, -0.3), di::State(1.0, 1.0, 0.3, 0.3));
    auto fmt = rrt::FastMarchingTree(start, goal, is_obstacle_free, sbound, 0.01, 4.0, 2000);
    // measure time
    auto start_time = std::chrono::system_clock::now();
    while(true){
      if(fmt.is_solved()){
        std::cout << "success" << std::endl;
        break;
      }
      if(fmt.is_failed()){
        std::cout << "failed" << std::endl;
        break;
      }
      fmt.extend();
    }
    auto end_time = std::chrono::system_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
    std::cout << "elapsed time: " << elapsed << "[ms]" << std::endl;

    // visualize all visited state by scattering
    {
      std::vector<double> xs, ys;
      for(const auto& node: fmt.nodes_) { 
        if(node->status != rrt::FMTNodeStatus::UNVISITED){
          xs.push_back(node->state.x(0));
          ys.push_back(node->state.x(1));
        }
      }
      plt::scatter(xs, ys);
    }

    // show all edges
    {
      for(const auto& node: fmt.nodes_) { 
        if(node->status != rrt::FMTNodeStatus::UNVISITED && node->parent != nullptr){
          std::vector<double> xs;
          std::vector<double> ys;
          auto traj = di::TrajectoryPiece(node->parent->state, node->state, *node->duration_from_parent);
          for(double t = 0.0; t < *node->duration_from_parent; t += fmt.dt_){
            auto state = traj.interpolate(t);
            xs.push_back(state.x(0));
            ys.push_back(state.x(1));
          }
          xs.push_back(node->state.x(0));
          ys.push_back(node->state.x(1));
          std::map<std::string, std::string> keywords;
          keywords["color"] = "k";
          keywords["linewidth"] = "0.3";
          plt::plot(xs, ys, keywords);
        }
      }
    }

    // show solution path
    auto solution = fmt.get_solution();
    {
      std::vector<double> xs;
      std::vector<double> ys;
      for(const auto& node: solution) { 
        if(node != nullptr && node->parent != nullptr){
          // create traj
          auto traj = di::TrajectoryPiece(node->parent->state, node->state, *node->duration_from_parent);
          for(double t = 0.0; t < *node->duration_from_parent; t += fmt.dt_){
            auto state = traj.interpolate(t);
            xs.push_back(state.x(0));
            ys.push_back(state.x(1));
          }
          xs.push_back(node->state.x(0));
          ys.push_back(node->state.x(1));
          std::map<std::string, std::string> keywords;
          keywords["color"] = "b";
          keywords["linewidth"] = "1.0";
          plt::plot(xs, ys, keywords);
        }
      }

    }
    plt::show();
}
