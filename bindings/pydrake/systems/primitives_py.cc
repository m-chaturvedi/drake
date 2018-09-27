#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/systems/systems_pybind.h"
#include "drake/bindings/pydrake/util/cpp_template_pybind.h"
#include "drake/bindings/pydrake/util/drake_optional_pybind.h"
#include "drake/systems/primitives/adder.h"
#include "drake/systems/primitives/affine_system.h"
#include "drake/systems/primitives/barycentric_system.h"
#include "drake/systems/primitives/constant_value_source.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/demultiplexer.h"
#include "drake/systems/primitives/gain.h"
#include "drake/systems/primitives/integrator.h"
#include "drake/systems/primitives/linear_system.h"
#include "drake/systems/primitives/matrix_gain.h"
#include "drake/systems/primitives/multiplexer.h"
#include "drake/systems/primitives/pass_through.h"
#include "drake/systems/primitives/random_source.h"
#include "drake/systems/primitives/saturation.h"
#include "drake/systems/primitives/signal_logger.h"
#include "drake/systems/primitives/trajectory_source.h"
#include "drake/systems/primitives/wrap_to_system.h"
#include "drake/systems/primitives/zero_order_hold.h"

#define D(...) DOC(drake, systems, __VA_ARGS__)
namespace drake {
namespace pydrake {

PYBIND11_MODULE(primitives, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems;

  m.doc() = "Bindings for the primitives portion of the Systems framework.";

  py::module::import("pydrake.systems.framework");

  auto bind_common_scalar_types = [m](auto dummy) {
    using T = decltype(dummy);

    DefineTemplateClassWithDefault<Adder<T>, LeafSystem<T>>(
        m, "Adder", GetPyParam<T>())
        .def(py::init<int, int>(), py::arg("num_inputs"), py::arg("size"),
        D(Adder));

    DefineTemplateClassWithDefault<AffineSystem<T>, LeafSystem<T>>(
        m, "AffineSystem", GetPyParam<T>())
        .def(py::init<const Eigen::Ref<const Eigen::MatrixXd>&,
                      const Eigen::Ref<const Eigen::MatrixXd>&,
                      const Eigen::Ref<const Eigen::VectorXd>&,
                      const Eigen::Ref<const Eigen::MatrixXd>&,
                      const Eigen::Ref<const Eigen::MatrixXd>&,
                      const Eigen::Ref<const Eigen::VectorXd>&, double>(),
             py::arg("A"), py::arg("B"), py::arg("f0"), py::arg("C"),
             py::arg("D"), py::arg("y0"), py::arg("time_period") = 0.0,
             D(AffineSystem))
        // TODO(eric.cousineau): Fix these to return references instead of
        // copies.
        .def("A", overload_cast_explicit<const Eigen::MatrixXd&>(
            &AffineSystem<T>::A), D(AffineSystem, A))
        .def("B", overload_cast_explicit<const Eigen::MatrixXd&>(
            &AffineSystem<T>::B), D(AffineSystem, B))
        .def("f0", overload_cast_explicit<const Eigen::VectorXd&>(
            &AffineSystem<T>::f0), D(AffineSystem, f0))
        .def("C", overload_cast_explicit<const Eigen::MatrixXd&>(
            &AffineSystem<T>::C), D(DefineTemplateClassWithDefault, C))
        .def("D", overload_cast_explicit<const Eigen::MatrixXd&>(
            &AffineSystem<T>::D), D(DefineTemplateClassWithDefault, D))
        .def("y0", overload_cast_explicit<const Eigen::VectorXd&>(
            &AffineSystem<T>::y0), D(DefineTemplateClassWithDefault, y0))
        .def("time_period", &AffineSystem<T>::time_period,
            D(DefineTemplateClassWithDefault, time_period));

    DefineTemplateClassWithDefault<ConstantValueSource<T>, LeafSystem<T>>(
        m, "ConstantValueSource", GetPyParam<T>(),
            D(DefineTemplateClassWithDefault, ConstantValueSource));

    DefineTemplateClassWithDefault<ConstantVectorSource<T>, LeafSystem<T>>(
        m, "ConstantVectorSource", GetPyParam<T>(), D(ConstantValueSource))
        .def(py::init<VectorX<T>>(),
             D(ConstantValueSource, ConstantValueSource));

    DefineTemplateClassWithDefault<Demultiplexer<T>, LeafSystem<T>>(
        m, "Demultiplexer", GetPyParam<T>(), D(Demultiplexer))
        .def(py::init<int, int>(),
             py::arg("size"),
             py::arg("output_ports_sizes") = 1,
             D(Demultiplexer, Demultiplexer));

    DefineTemplateClassWithDefault<Gain<T>, LeafSystem<T>>(
        m, "Gain", GetPyParam<T>(), D(Gain))
        .def(py::init<double, int>(), py::arg("k"), py::arg("size"),
             D(Gain, Gain, 3))
        .def(py::init<const Eigen::Ref<const Eigen::VectorXd>&>(),
             py::arg("k"), D(Gain, Gain, 4));

    DefineTemplateClassWithDefault<Integrator<T>, LeafSystem<T>>(
        m, "Integrator", GetPyParam<T>())
        .def(py::init<int>(), D(Integrator, Integrator, 3));

    DefineTemplateClassWithDefault<LinearSystem<T>, AffineSystem<T>>(
        m, "LinearSystem", GetPyParam<T>(), D(LinearSystem))
        .def(py::init<const Eigen::Ref<const Eigen::MatrixXd>&,
                      const Eigen::Ref<const Eigen::MatrixXd>&,
                      const Eigen::Ref<const Eigen::MatrixXd>&,
                      const Eigen::Ref<const Eigen::MatrixXd>&, double>(),
             py::arg("A"), py::arg("B"), py::arg("C"), py::arg("D"),
             py::arg("time_period") = 0.0, D(LinearSystem, time_period));

    DefineTemplateClassWithDefault<MatrixGain<T>, LinearSystem<T>>(
        m, "MatrixGain", GetPyParam<T>(), D(MatrixGain))
        .def(py::init<const Eigen::Ref<const Eigen::MatrixXd>&>(),
             py::arg("D"), D(MatrixGain, MatrixGain));

    DefineTemplateClassWithDefault<Multiplexer<T>, LeafSystem<T>>(
        m, "Multiplexer", GetPyParam<T>(), D(Multiplexer))
        .def(py::init<int>(), py::arg("num_scalar_inputs"),
             D(Multiplexer, Multiplexer))
        .def(py::init<std::vector<int>>(), py::arg("input_sizes"),
             D(Multiplexer, Multiplexer))
        .def(py::init<const BasicVector<T>&>(), py::arg("model_vector"),
             D(Multiplexer, Multiplexer));

    DefineTemplateClassWithDefault<PassThrough<T>, LeafSystem<T>>(
        m, "PassThrough", GetPyParam<T>(), D(PassThrough))
        .def(py::init<int>(), D(PassThrough, PassThrough, 3))
        .def(py::init<const AbstractValue&>(), D(PassThrough, PassThrough, 4));

    DefineTemplateClassWithDefault<Saturation<T>, LeafSystem<T>>(
        m, "Saturation", GetPyParam<T>(), D(Saturation))
        .def(py::init<const VectorX<T>&, const VectorX<T>&>(), py::arg
      ("min_value"), py::arg("max_value"), D(Saturation, Saturation));

    DefineTemplateClassWithDefault<SignalLogger<T>, LeafSystem<T>>(
        m, "SignalLogger", GetPyParam<T>(), D(SignalLogger))
        .def(py::init<int>(), D(SignalLogger, SignalLogger))
        .def(py::init<int, int>(), D(SignalLogger, SignalLogger, 3))
        .def("sample_times", &SignalLogger<T>::sample_times,
             D(SignalLogger, sample_times))
        .def("data", &SignalLogger<T>::data, D(SignalLogger, data))
        .def("reset", &SignalLogger<T>::reset, D(SignalLogger, reset));

    DefineTemplateClassWithDefault<WrapToSystem<T>, LeafSystem<T>>(
        m, "WrapToSystem", GetPyParam<T>(), D(WrapToSystem))
        .def(py::init<int>(), D(WrapToSystem, WrapToSystem))
        .def("set_interval", &WrapToSystem<T>::set_interval,
             D(WrapToSystem, set_interval));

    DefineTemplateClassWithDefault<ZeroOrderHold<T>, LeafSystem<T>>(
        m, "ZeroOrderHold", GetPyParam<T>())
        .def(py::init<double, int>(), D(ZeroOrderHold, ZeroOrderHold));
  };
  type_visit(bind_common_scalar_types, pysystems::CommonScalarPack{});

  py::class_<BarycentricMeshSystem<double>, LeafSystem<double>>(
      m, "BarycentricMeshSystem", D(BarycentricMeshSystem))
      .def(py::init<math::BarycentricMesh<double>,
                    const Eigen::Ref<const MatrixX<double>>&>(),
         D(BarycentricMeshSystem, BarycentricMeshSystem))
      .def("get_mesh", &BarycentricMeshSystem<double>::get_mesh,
         D(BarycentricMeshSystem, get_mesh))
      .def("get_output_values",
           &BarycentricMeshSystem<double>::get_output_values,
         D(BarycentricMeshSystem, get_output_values));

  py::class_<UniformRandomSource, LeafSystem<double>>(m, "UniformRandomSource",
    D(UniformRandomSource))
      .def(py::init<int, double>(), py::arg("num_outputs"),
           py::arg("sampling_interval_sec"),
        D(UniformRandomSource, UniformRandomSource));

  py::class_<GaussianRandomSource, LeafSystem<double>>(m,
                                                       "GaussianRandomSource",
      D(GaussianRandomSource))
      .def(py::init<int, double>(), py::arg("num_outputs"),
        py::arg("sampling_interval_sec"),
        D(GaussianRandomSource, GaussianRandomSource));

  py::class_<ExponentialRandomSource, LeafSystem<double>>(
      m, "ExponentialRandomSource", D(ExponentialRandomSource))
      .def(py::init<int, double>(), py::arg("num_outputs"),
           py::arg("sampling_interval_sec"),
        D(ExponentialRandomSource, ExponentialRandomSource));

  py::class_<TrajectorySource<double>, LeafSystem<double>>(
        m, "TrajectorySource", D(TrajectorySource))
        .def(py::init<const trajectories::Trajectory<double>&, int, bool>(),
          py::arg("trajectory"),
          py::arg("output_derivative_order") = 0,
          py::arg("zero_derivatives_beyond_limits") = true,
          D(TrajectorySource, TrajectorySource));

  m.def("AddRandomInputs", &AddRandomInputs, py::arg("sampling_interval_sec"),
        py::arg("builder"), D(TrajectorySource, AddRandomInputs));

  m.def("Linearize", &Linearize, py::arg("system"), py::arg("context"),
        py::arg("input_port_index") = systems::kUseFirstInputIfItExists,
        py::arg("output_port_index") = systems::kUseFirstOutputIfItExists,
        py::arg("equilibrium_check_tolerance") = 1e-6,
        D(TrajectorySource, Linearize));

  m.def("FirstOrderTaylorApproximation", &FirstOrderTaylorApproximation,
        py::arg("system"), py::arg("context"),
        py::arg("input_port_index") = systems::kUseFirstInputIfItExists,
        py::arg("output_port_index") = systems::kUseFirstOutputIfItExists,
        D(TrajectorySource, FirstOrderTaylorApproximation));

  m.def("ControllabilityMatrix", &ControllabilityMatrix,
    D(TrajectorySource, ControllabilityMatrix));

  m.def("IsControllable", &IsControllable, py::arg("sys"),
        py::arg("threshold") = nullopt, D(TrajectorySource, IsControllable));

  m.def("ObservabilityMatrix", &ObservabilityMatrix,
    D(TrajectorySource, ObservabilityMatrix));

  m.def("IsObservable", &IsObservable, py::arg("sys"),
        py::arg("threshold") = nullopt, D(TrajectorySource, IsObservable));

  m.def("LogOutput", &LogOutput<double>, py::arg("src"), py::arg("builder"),
        // Keep alive, ownership: `return` keeps `builder` alive.
        py::keep_alive<0, 2>(),
        // TODO(eric.cousineau): Figure out why this is necessary (#9398).
        py_reference, D(TrajectorySource, LogOutput));

  // TODO(eric.cousineau): Add more systems as needed.
}

}  // namespace pydrake
}  // namespace drake
