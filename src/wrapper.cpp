#include "sbmp.hpp"
#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;
using namespace double_integrator_planning;

PYBIND11_MODULE(_diplan, m) {
  m.doc() = "Samplinng based double integrator planning";
  py::class_<State>(m, "_State")
      .def(py::init<double, double, double, double>())
      .def("to_vector", &State::to_vector);

  py::class_<BoundingBox>(m, "_BoundingBox")
      .def(py::init<const State &, const State &>());

  py::class_<TrajectoryPiece>(m, "_TrajectoryPiece")
      .def(py::init<const State &, const State &, double>())
      .def_readonly("duration", &TrajectoryPiece::duration_)
      .def("interpolate", &TrajectoryPiece::interpolate);

  py::class_<FastMarchingTree>(m, "_FastMarchingTree")
      .def(py::init<const State &, const State &, std::function<bool(State)>,
                    const BoundingBox &, double, double, size_t>())
      .def("get_solution", &FastMarchingTree::get_solution)
      .def("get_all_motions", &FastMarchingTree::get_all_motions)
      .def("solve", &FastMarchingTree::solve);
}
