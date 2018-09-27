#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

#include "drake/automotive/calc_ongoing_road_position.h"
#include "drake/automotive/gen/driving_command.h"
#include "drake/automotive/idm_controller.h"
#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/pose_selector.h"
#include "drake/automotive/pure_pursuit_controller.h"
#include "drake/automotive/road_odometry.h"
#include "drake/automotive/simple_car.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/systems/systems_pybind.h"
#include "drake/bindings/pydrake/util/wrap_pybind.h"
#include "drake/systems/framework/basic_vector.h"
#include drake/bindings/pydrake/documentation_pybind.h
#define D(...) DOC(drake, automotive, __VA_ARGS__)

namespace drake {
namespace pydrake {

PYBIND11_MODULE(automotive, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems;
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::automotive;

  m.doc() = "Bindings for Automotive systems";

  py::module::import("pydrake.systems.framework");
  py::module::import("pydrake.systems.rendering");

  using T = double;

  py::enum_<AheadOrBehind>(m, "AheadOrBehind", D(AheadOrBehind))
      .value("kAhead", AheadOrBehind::kAhead, D(AheadOrBehind, kAhead))
      .value("kBehind", AheadOrBehind::kBehind), D(AheadOrBehind, kBehind);

  py::enum_<RoadPositionStrategy>(m, "RoadPositionStrategy",
      D(RoadPositionStrategy))
      .value("kCache", RoadPositionStrategy::kCache,
      D(RoadPositionStrategy, kCache))
      .value("kExhaustiveSearch", RoadPositionStrategy::kExhaustiveSearch,
      D(RoadPositionStrategy, kExhaustiveSearch));

  py::enum_<ScanStrategy>(m, "ScanStrategy", D(ScanStrategy))
      .value("kPath", ScanStrategy::kPath, D(ScanStrategy, kPath))
      .value("kBranches", ScanStrategy::kBranches, D(ScanStrategy, kBranches));

  py::class_<ClosestPose<T>>(m, "ClosestPose", D(ClosestPose))
      .def(py::init<>(), D(ClosestPose, ClosestPose))
      .def(py::init<const RoadOdometry<T>&, const T&>(), py::arg("odom"),
           py::arg("dist"),
           // Keep alive, transitive: `self` keeps `RoadOdometry` pointer
           // members alive.
           py::keep_alive<1, 2>(), D(ClosestPose, ClosestPose, 2))
      .def_readwrite("odometry", &ClosestPose<T>::odometry,
                     py_reference_internal, D(ClosestPose, odometry))
      .def_readwrite("distance", &ClosestPose<T>::distance,
                D(ClosestPose, distance));

  py::class_<RoadOdometry<T>> road_odometry(m, "RoadOdometry",
    D(RoadOdometry));
  road_odometry
      .def(py::init<>(), D(RoadOdometry, RoadOdometry))
      .def(py::init<const maliput::api::RoadPosition&,
                    const systems::rendering::FrameVelocity<T>&>(),
           py::arg("road_position"), py::arg("frame_velocity"),
           // Keep alive, transitive: `self` keeps `RoadPosition` pointer
           // members alive.
           py::keep_alive<1, 2>(), D(RoadOdometry, RoadOdometry, 2))
      .def(py::init<const maliput::api::Lane*,
                    const maliput::api::LanePositionT<T>&,
                    const systems::rendering::FrameVelocity<T>&>(),
           py::arg("lane"), py::arg("lane_position"), py::arg("frame_velocity"),
           // Keep alive, reference: `self` keeps `Lane*` alive.
           py::keep_alive<1, 2>(), D(RoadOdometry, RoadOdometry, 3))
      .def_readwrite("pos", &RoadOdometry<T>::pos,
           D(RoadOdometry, pos))
      .def_readwrite("vel", &RoadOdometry<T>::vel,
           D(RoadOdometry, vel));
  DefReadWriteKeepAlive(&road_odometry, "lane", &RoadOdometry<T>::lane);

  py::class_<LaneDirection>(m, "LaneDirection", D(LaneDirection))
      .def(py::init<const maliput::api::Lane*, bool>(), py::arg("lane"),
           py::arg("with_s"), D(LaneDirection, LaneDirection))
      .def_readwrite("lane", &LaneDirection::lane, py_reference_internal,
           D(LaneDirection, lane))
      .def_readwrite("with_s", &LaneDirection::with_s,
           D(LaneDirection, with_s));
  pysystems::AddValueInstantiation<LaneDirection>(m);

  // TODO(eric.cousineau) Bind this named vector automatically (see #8096).
  py::class_<DrivingCommand<T>, BasicVector<T>>(m, "DrivingCommand",
      D(DrivingCommand))
      .def(py::init<>(), D(DrivingCommand, DrivingCommand))
      .def("steering_angle", &DrivingCommand<T>::steering_angle,
      D(DrivingCommand, steering_angle))
      .def("acceleration", &DrivingCommand<T>::acceleration,
      D(DrivingCommand, acceleration))
      .def("set_steering_angle", &DrivingCommand<T>::set_steering_angle,
      D(DrivingCommand, set_steering_angle))
      .def("set_acceleration", &DrivingCommand<T>::set_acceleration,
      D(DrivingCommand, set_acceleration));

  py::class_<IdmController<T>, LeafSystem<T>>(m, "IdmController", D(IdmController))
      .def(py::init<const maliput::api::RoadGeometry&, ScanStrategy,
                    RoadPositionStrategy, double>(),
           py::arg("road"), py::arg("path_or_branches"),
           py::arg("road_position_strategy"), py::arg("period_sec"),
           D(IdmController, IdmController))
      .def("ego_pose_input", &IdmController<T>::ego_pose_input,
           py_reference_internal, D(IdmController, ego_pose_input))
      .def("ego_velocity_input", &IdmController<T>::ego_velocity_input,
           py_reference_internal, D(IdmController, ego_velocity_input))
      .def("traffic_input", &IdmController<T>::traffic_input,
           py_reference_internal, D(IdmController, traffic_input))
      .def("acceleration_output", &IdmController<T>::acceleration_output,
           py_reference_internal, D(IdmController, acceleration_output));

  py::class_<PoseSelector<T>>(m, "PoseSelector", D(PoseSelector))
      .def_static(
          "FindClosestPair",
          [](const maliput::api::Lane* lane,
             const systems::rendering::PoseVector<T>& ego_pose,
             const systems::rendering::PoseBundle<T>& traffic_poses,
             const T& scan_distance, ScanStrategy path_or_branches) {
            return PoseSelector<T>::FindClosestPair(
                lane, ego_pose, traffic_poses, scan_distance, path_or_branches);
          },
          py::arg("lane"), py::arg("ego_pose"), py::arg("traffic_poses"),
          py::arg("scan_distance"), py::arg("path_or_branches"),
          D(PoseSelector, FindClosestPair))
      .def_static("FindSingleClosestPose",
                  [](const maliput::api::Lane* lane,
                     const systems::rendering::PoseVector<T>& ego_pose,
                     const systems::rendering::PoseBundle<T>& traffic_poses,
                     const T& scan_distance, const AheadOrBehind side,
                     ScanStrategy path_or_branches) {
                    return PoseSelector<T>::FindSingleClosestPose(
                        lane, ego_pose, traffic_poses, scan_distance, side,
                        path_or_branches);
                  },
                  py::arg("lane"), py::arg("ego_pose"),
                  py::arg("traffic_poses"), py::arg("scan_distance"),
                  py::arg("side"), py::arg("path_or_branches"),
                  D(PoseSelector, FindSingleClosestPose))
      .def_static("GetSigmaVelocity", &PoseSelector<T>::GetSigmaVelocity,
                  D(PoseSelector, GetSigmaVelocity));

  py::class_<PurePursuitController<T>, LeafSystem<T>>(m,
                                                      "PurePursuitController",
        D(PurePursuitController))
      .def(py::init<>(), D(PurePursuitController, PurePursuitController))
      .def("ego_pose_input", &PurePursuitController<T>::ego_pose_input,
           py_reference_internal, D(PurePursuitController, ego_pose_input))
      .def("lane_input", &PurePursuitController<T>::lane_input,
           py_reference_internal, D(PurePursuitController, lane_input))
      .def("steering_command_output",
           &PurePursuitController<T>::steering_command_output,
           py_reference_internal, D(PurePursuitController, steering_command_input));

  // TODO(eric.cousineau) Bind this named vector automatically (see #8096).
  py::class_<SimpleCarState<T>, BasicVector<T>>(m, "SimpleCarState", D(SimpleCarState))
      .def(py::init<>(), D(SimpleCarState, SimpleCarState))
      .def("x", &SimpleCarState<T>::x, D(SimpleCarState, x))
      .def("y", &SimpleCarState<T>::y, D(SimpleCarState, y))
      .def("heading", &SimpleCarState<T>::heading, D(SimpleCarState, heading))
      .def("velocity", &SimpleCarState<T>::velocity,
        D(SimpleCarState, velocity))
      .def("set_x", &SimpleCarState<T>::set_x, D(SimpleCarState, set_x))
      .def("set_y", &SimpleCarState<T>::set_y, D(SimpleCarState, set_y))
      .def("set_heading", &SimpleCarState<T>::set_heading,
        D(SimpleCarState, set_heading))
      .def("set_velocity", &SimpleCarState<T>::set_velocity,
        D(SimpleCarState, set_velocity));

  py::class_<SimpleCar<T>, LeafSystem<T>>(m, "SimpleCar", D(SimpleCar))
      .def(py::init<>(), D(SimpleCar, SimpleCar))
      .def("state_output", &SimpleCar<T>::state_output, py_reference_internal,
        D(SimpleCar, state_output))
      .def("pose_output", &SimpleCar<T>::pose_output, py_reference_internal,
        D(SimpleCar, pose_output))
      .def("velocity_output", &SimpleCar<T>::velocity_output,
           py_reference_internal, D(SimpleCar, velocity_output));

  // TODO(jadecastro) Bind more systems as appropriate.
}

}  // namespace pydrake
}  // namespace drake
