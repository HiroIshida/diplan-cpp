#include "sbmp.hpp"
#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;
using namespace double_integrator_planning;

PYBIND11_MODULE(_disbmp, m) {
  m.doc() = "Samplinng based double integrator planning";
  py::class_<State>(m, "_State")
      .def(py::init<double, double, double, double>())
      .def("to_vector", &State::to_vector);

  py::class_<BoundingBox>(m, "_BoundingBox")
      .def(py::init<const State &, const State &>());

  py::class_<TrajectoryPiece>(m, "_TrajectoryPiece")
      .def(py::init<double const, Eigen::Vector4d const &,
                    Eigen::Vector4d const &>())
      .def(py::init<const State &, const State &, double>())
      .def_readonly("duration", &TrajectoryPiece::duration)
      .def_readonly("d", &TrajectoryPiece::d)
      .def_readonly("s1_vec", &TrajectoryPiece::s1_vec)
      .def("interpolate", &TrajectoryPiece::interpolate);

  py::class_<Trajectory>(m, "_Trajectory")
      .def(py::init<std::vector<TrajectoryPiece>>())
      .def_readonly("pieces", &Trajectory::pieces)
      .def("get_duration", &Trajectory::get_duration)
      .def("get_length", &Trajectory::get_length)
      .def("interpolate", &Trajectory::interpolate);

  py::class_<RRT>(m, "_RRT")
      .def(py::init<const State &, const State &, std::function<bool(State)>,
                    const BoundingBox &, double, double>())
      .def("solve", &RRT::solve)
      .def("get_solution", &RRT::get_solution);

  py::class_<FastMarchingTree>(m, "_FastMarchingTree")
      .def(py::init<const State &, const State &, std::function<bool(State)>,
                    const BoundingBox &, double, double, size_t>())
      .def("get_solution", &FastMarchingTree::get_solution)
      .def("get_all_motions", &FastMarchingTree::get_all_motions)
      .def("solve", &FastMarchingTree::solve);
}
