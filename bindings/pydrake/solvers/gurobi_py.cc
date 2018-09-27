#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/solvers/gurobi_solver.h"

#define D(...) DOC(drake, solvers, MathematicalProgam, __VA_ARGS__)
namespace drake {
namespace pydrake {

PYBIND11_MODULE(gurobi, m) {
  using drake::solvers::GurobiSolver;

  m.doc() = "Gurobi solver bindings for MathematicalProgram";

  py::object solverinterface =
      py::module::import("pydrake.solvers.mathematicalprogram").attr(
          "MathematicalProgramSolverInterface");

  py::class_<GurobiSolver>(m, "GurobiSolver", solverinterface)
    .def(py::init<>(solvers, GurobiSolver));
}

}  // namespace pydrake
}  // namespace drake
