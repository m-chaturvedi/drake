#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/systems/systems_pybind.h"
#include "drake/systems/analysis/integrator_base.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/systems/analysis/runge_kutta3_integrator.h"
#include "drake/systems/analysis/simulator.h"

#define D(...) DOC(drake, systems, __VA_ARGS__)
using std::unique_ptr;

namespace drake {
namespace pydrake {

PYBIND11_MODULE(analysis, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems;

  m.doc() = "Bindings for the analysis portion of the Systems framework.";

  py::module::import("pydrake.systems.framework");

  auto bind_scalar_types = [m](auto dummy) {
    using T = decltype(dummy);
    DefineTemplateClassWithDefault<IntegratorBase<T>>(
        m, "IntegratorBase", GetPyParam<T>())
      .def("set_fixed_step_mode", &IntegratorBase<T>::set_fixed_step_mode,
      D(IntegratorBase, set_fixed_step_mode))
      .def("get_fixed_step_mode", &IntegratorBase<T>::get_fixed_step_mode,
      D(IntegratorBase, get_fixed_step_mode))
      .def("set_target_accuracy", &IntegratorBase<T>::set_target_accuracy,
      D(IntegratorBase, set_target_accuracy))
      .def("get_target_accuracy", &IntegratorBase<T>::get_target_accuracy,
      D(IntegratorBase, get_target_accuracy))
      .def("set_maximum_step_size", &IntegratorBase<T>::set_maximum_step_size,
      D(IntegratorBase, set_maximum_step_size))
      .def("get_maximum_step_size", &IntegratorBase<T>::get_maximum_step_size,
      D(IntegratorBase, get_maximum_step_size))
      .def("set_requested_minimum_step_size",
           &IntegratorBase<T>::set_requested_minimum_step_size,
      D(IntegratorBase, set_requested_minimum_step_size))
      .def("get_requested_minimum_step_size",
           &IntegratorBase<T>::get_requested_minimum_step_size,
      D(IntegratorBase, get_requested_minimum_step_size))
      .def("set_throw_on_minimum_step_size_violation",
           &IntegratorBase<T>::set_throw_on_minimum_step_size_violation,
      D(IntegratorBase, set_throw_on_minimum_step_size_violation))
      .def("get_throw_on_minimum_step_size_violation",
           &IntegratorBase<T>::get_throw_on_minimum_step_size_violation,
      D(IntegratorBase, get_throw_on_minimum_step_size_violation));

    DefineTemplateClassWithDefault<RungeKutta2Integrator<T>,
                                   IntegratorBase<T>>(
       m, "RungeKutta2Integrator", GetPyParam<T>())
      .def(py::init<const System<T>&, const T&>(),
           py::arg("system"),
           py::arg("max_step_size"),
           // Keep alive, reference: `self` keeps `System` alive.
           py::keep_alive<1, 2>(),
               D(RungeKutta2Integrator, RungeKutta2Integrator, 2))
      .def(py::init<const System<T>&, const T&, Context<T>*>(),
           py::arg("system"),
           py::arg("max_step_size"),
           py::arg("context"),
           // Keep alive, reference: `self` keeps `System` alive.
           py::keep_alive<1, 2>(),
           // Keep alive, reference: `self` keeps `Context` alive.
           py::keep_alive<1, 4>(),
           D(RungeKutta2Integrator, RungeKutta2Integrator, 3));

    DefineTemplateClassWithDefault<RungeKutta3Integrator<T>,
                                   IntegratorBase<T>>(
       m, "RungeKutta3Integrator", GetPyParam<T>())
      .def(py::init<const System<T>&>(),
           // Keep alive, reference: `self` keeps `System` alive.
           py::arg("system"),
           py::keep_alive<1, 2>(),
               D(RungeKutta3Integrator, RungeKutta3Integrator))
      .def(py::init<const System<T>&, Context<T>*>(),
           py::arg("system"),
           py::arg("context"),
           // Keep alive, reference: `self` keeps `System` alive.
           py::keep_alive<1, 2>(),
           // Keep alive, reference: `self` keeps `Context` alive.
           py::keep_alive<1, 3>(),
               D(RungeKutta3Integrator, RungeKutta3Integrator, 3));

    DefineTemplateClassWithDefault<Simulator<T>>(
        m, "Simulator", GetPyParam<T>(), D(Simulator))
      .def(py::init<const System<T>&>(),
           py::arg("system"),
           // Keep alive, reference: `self` keeps `System` alive.
           py::keep_alive<1, 2>(), D(Simulator, Simulator, 3))
      .def(py::init<const System<T>&, unique_ptr<Context<T>>>(),
           py::arg("system"),
           py::arg("context"),
           // Keep alive, reference: `self` keeps `System` alive.
           py::keep_alive<1, 2>(),
           // Keep alive, ownership: `Context` keeps `self` alive.
           py::keep_alive<3, 1>(), D(Simulator, Simulator, 3))
      .def("Initialize", &Simulator<T>::Initialize, D(Simulator, Initialize))
      .def("StepTo", &Simulator<T>::StepTo, D(Simulator, StepTo))
      .def("get_context", &Simulator<T>::get_context, py_reference_internal,
           D(Simulator, get_context))
      .def("get_integrator", &Simulator<T>::get_integrator,
           py_reference_internal, D(Simulator, get_integrator))
      .def("get_mutable_integrator", &Simulator<T>::get_mutable_integrator,
           py_reference_internal, D(Simulator, get_mutable_integrator))
      .def("get_mutable_context", &Simulator<T>::get_mutable_context,
           py_reference_internal, D(Simulator, get_mutable_context))
      .def("reset_integrator",
           [](Simulator<T>* self,
              std::unique_ptr<IntegratorBase<T>> integrator) {
             return self->reset_integrator(std::move(integrator));
           },
           // Keep alive, ownership: 'Integrator' keeps 'self' alive.
           py::keep_alive<2, 1>(), D(Simulator, reset_integrator))
      .def("set_publish_every_time_step",
           &Simulator<T>::set_publish_every_time_step,
           D(Simulator, set_publish_every_time_step))
      .def("set_target_realtime_rate", &Simulator<T>::set_target_realtime_rate,
           D(Simulator, set_target_realtime_rate));
  };
  type_visit(bind_scalar_types, pysystems::NonSymbolicScalarPack{});
}

}  // namespace pydrake
}  // namespace drake
