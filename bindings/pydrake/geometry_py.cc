#include "pybind11/operators.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/scene_graph.h"
#define D(...) DOC(drake, geometry, __VA_ARGS__)

namespace drake {
namespace pydrake {
namespace {

using systems::LeafSystem;

// TODO(eric.cousineau): Bind additional scalar types.
using T = double;

template <typename Class>
void BindIdentifier(py::module m, const std::string& name) {
  py::class_<Class>(m, name.c_str())
      .def(py::init<>())
      .def("get_value", &Class::get_value)
      .def("is_valid", &Class::is_valid)
      .def(py::self == py::self)
      .def(py::self != py::self)
      .def(py::self < py::self);
}

PYBIND11_MODULE(geometry, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::geometry;

  py::module::import("pydrake.systems.framework");
  py::class_<SceneGraph<T>, LeafSystem<T>>(m, "SceneGraph", D(SceneGraph))
      .def(py::init<>(), D(SceneGraph, SceneGraph))
      .def("get_source_pose_port", &SceneGraph<T>::get_source_pose_port,
           py_reference_internal, D(SceneGraph, get_source_pose_port))
      .def("get_pose_bundle_output_port",
           &SceneGraph<T>::get_pose_bundle_output_port, py_reference_internal,
           D(SceneGraph, get_pose_bundle_output_port))
      .def("get_query_output_port", &SceneGraph<T>::get_query_output_port,
           py_reference_internal, D(SceneGraph, get_query_output_port));

  BindIdentifier<SourceId>(m, "SourceId");
  BindIdentifier<FrameId>(m, "FrameId");
  BindIdentifier<GeometryId>(m, "GeometryId");

  m.def("ConnectDrakeVisualizer", &ConnectDrakeVisualizer,
        py::arg("builder"), py::arg("scene_graph"), py::arg("lcm") = nullptr,
        D(ConnectDrakeVisualizer));
  m.def("DispatchLoadMessage", &DispatchLoadMessage,
        py::arg("scene_graph"), py::arg("lcm"), D(DispatchLoadMessage));
}

}  // namespace
}  // namespace pydrake
}  // namespace drake
