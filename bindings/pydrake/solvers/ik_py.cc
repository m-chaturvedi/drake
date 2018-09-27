#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/multibody/ik_options.h"
#include "drake/multibody/rigid_body_ik.h"
#include "drake/multibody/rigid_body_tree.h"
#define D(...) DOC(__VA_ARGS__)

namespace drake {
namespace pydrake {

PYBIND11_MODULE(_ik_py, m) {
  m.doc() = "RigidBodyTree inverse kinematics";

  py::class_<RigidBodyConstraint>(m, "RigidBodyConstraint",
    D(RigidBodyConstraint));

  py::class_<PostureConstraint, RigidBodyConstraint>(m, "PostureConstraint",
    D(PostureConstraint))
    .def(py::init<RigidBodyTree<double> *,
                  const Eigen::Vector2d& >(),
         py::arg("model"),
         py::arg("tspan") = DrakeRigidBodyConstraint::default_tspan,
         D(PostureConstraint, RigidBodyTree))
    .def("setJointLimits",
         static_cast<void(PostureConstraint::*)(
             const Eigen::VectorXi&,
             const Eigen::VectorXd&,
             const Eigen::VectorXd&)>(
                 &PostureConstraint::setJointLimits),
        D(PostureConstraint, setJointLimits));

  py::class_<WorldPositionConstraint, RigidBodyConstraint>(
    m, "WorldPositionConstraint", D(WorldPositionConstraint))
    .def(py::init<RigidBodyTree<double>*,
                  int,
                  const Eigen::Matrix3Xd&,
                  Eigen::MatrixXd,
                  Eigen::MatrixXd,
                  const Eigen::Vector2d&>(),
         py::arg("model"),
         py::arg("body"),
         py::arg("pts"),
         py::arg("lb"),
         py::arg("ub"),
         py::arg("tspan") = DrakeRigidBodyConstraint::default_tspan,
         D(WorldPositionConstraint, WorldPositionConstraint));

  py::class_<RelativePositionConstraint, RigidBodyConstraint>(
    m, "RelativePositionConstraint", D(RelativePositionConstraint))
    .def(py::init<RigidBodyTree<double>*,
                  const Eigen::Matrix3Xd&,
                  const Eigen::MatrixXd&,
                  const Eigen::MatrixXd&,
                  int,
                  int,
                  const Eigen::Matrix<double, 7, 1>&,
                  const Eigen::Vector2d&>(),
         py::arg("model"),
         py::arg("pts"),
         py::arg("lb"),
         py::arg("ub"),
         py::arg("bodyA_idx"),
         py::arg("bodyB_idx"),
         py::arg("bTbp"),
         py::arg("tspan") = DrakeRigidBodyConstraint::default_tspan,
         D(RelativePositionConstraint, RelativePositionConstraint));

  py::class_<RelativeQuatConstraint, RigidBodyConstraint>(
    m, "RelativeQuatConstraint", D(RelativeQuatConstraint))
    .def(py::init<RigidBodyTree<double>*,
                  int,
                  int,
                  const Eigen::Vector4d&,
                  double,
                  const Eigen::Vector2d&>(),
         py::arg("model"),
         py::arg("bodyA_idx"),
         py::arg("bodyB_idx"),
         py::arg("quat_des"),
         py::arg("tol"),
         py::arg("tspan") = DrakeRigidBodyConstraint::default_tspan,
         D(RelativeQuatConstraint, RelativeQuatConstraint));

  py::class_<WorldPositionInFrameConstraint, RigidBodyConstraint>(
    m, "WorldPositionInFrameConstraint", D(WorldPositionInFrameConstraint))
    .def(py::init<RigidBodyTree<double>*,
                  int,
                  const Eigen::Matrix3Xd&,
                  const Eigen::Matrix4d&,
                  const Eigen::MatrixXd&,
                  const Eigen::MatrixXd&,
                  const Eigen::Vector2d&>(),
         py::arg("model"),
         py::arg("body"),
         py::arg("pts"),
         py::arg("T_world_to_frame"),
         py::arg("lb"),
         py::arg("ub"),
         py::arg("tspan") = DrakeRigidBodyConstraint::default_tspan,
         D(WorldPositionInFrameConstraint, WorldPositionInFrameConstraint));

  py::class_<WorldGazeDirConstraint, RigidBodyConstraint>(
    m, "WorldGazeDirConstraint", D(WorldGazeDirConstraint))
    .def(py::init<RigidBodyTree<double>*,
                  int,
                  const Eigen::Vector3d&,
                  const Eigen::Vector3d&,
                  double,
                  const Eigen::Vector2d&>(),
         py::arg("model"),
         py::arg("body"),
         py::arg("axis"),
         py::arg("dir"),
         py::arg("conethreshold"),
         py::arg("tspan") = DrakeRigidBodyConstraint::default_tspan,
         D(WorldGazeDirConstraint, WorldGazeDirConstraint));

  py::class_<WorldGazeTargetConstraint, RigidBodyConstraint>(
    m, "WorldGazeTargetConstraint", D(WorldGazeTargetConstraint))
    .def(py::init<RigidBodyTree<double>*,
                  int,
                  const Eigen::Vector3d&,
                  const Eigen::Vector3d&,
                  const Eigen::Vector3d&,
                  double,
                  const Eigen::Vector2d&>(),
        py::arg("model"),
        py::arg("body"),
        py::arg("axis"),
        py::arg("target"),
        py::arg("gaze_origin"),
        py::arg("conethreshold"),
        py::arg("tspan") = DrakeRigidBodyConstraint::default_tspan,
        D(WorldGazeTargetConstraint, WorldGazeTargetConstraint));

  py::class_<RelativeGazeDirConstraint, RigidBodyConstraint>(
    m, "RelativeGazeDirConstraint", D(RelativeGazeDirConstraint))
    .def(py::init<RigidBodyTree<double>*,
                  int,
                  int,
                  const Eigen::Vector3d&,
                  const Eigen::Vector3d&,
                  double,
                  const Eigen::Vector2d&>(),
         py::arg("model"),
         py::arg("bodyA_idx"),
         py::arg("bodyB_idx"),
         py::arg("axis"),
         py::arg("dir"),
         py::arg("conethreshold"),
         py::arg("tspan") = DrakeRigidBodyConstraint::default_tspan,
         D(RelativeGazeDirConstraint, RelativeGazeDirConstraint));

  py::class_<MinDistanceConstraint, RigidBodyConstraint>(
    m, "MinDistanceConstraint", D(MinDistanceConstraint))
    .def(py::init<RigidBodyTree<double>*,
                  double,
                  const std::vector<int>&,
                  const std::set<std::string>&,
                  const Eigen::Vector2d&>(),
         py::arg("model"),
         py::arg("min_distance"),
         py::arg("active_bodies_idx"),
         py::arg("active_group_names"),
         py::arg("tspan") = DrakeRigidBodyConstraint::default_tspan,
         D(MinDistanceConstraint, MinDistanceConstraint));

  py::class_<WorldEulerConstraint, RigidBodyConstraint>(
    m, "WorldEulerConstraint", D(WorldEulerConstraint))
    .def(py::init<RigidBodyTree<double>*,
                  int,
                  const Eigen::Vector3d&,
                  const Eigen::Vector3d&,
                  const Eigen::Vector2d>(),
         py::arg("model"),
         py::arg("body"),
         py::arg("lb"),
         py::arg("ub"),
         py::arg("tspan") = DrakeRigidBodyConstraint::default_tspan,
         D(WorldEulerConstraint, WorldEulerConstraint));

  py::class_<WorldQuatConstraint, RigidBodyConstraint>(
    m, "WorldQuatConstraint", D(WorldQuatConstraint))
    .def(py::init<RigidBodyTree<double>*,
                  int,
                  const Eigen::Vector4d&,
                  double,
                  const Eigen::Vector2d>(),
         py::arg("model"),
         py::arg("body"),
         py::arg("quat_des"),
         py::arg("tol"),
         py::arg("tspan") = DrakeRigidBodyConstraint::default_tspan,
         D(WorldQuatConstraint, WorldQuatConstraint));

  py::class_<QuasiStaticConstraint, RigidBodyConstraint>(
    m, "QuasiStaticConstraint", D(QuasiStaticConstraint))
    .def(py::init<RigidBodyTree<double>*, const Eigen::Vector2d&>(),
        py::arg("model"),
        py::arg("tspan") = DrakeRigidBodyConstraint::default_tspan,
        D(QuasiStaticConstraint, QuasiStaticConstraint))
    .def(py::init<RigidBodyTree<double>*,
                  const Eigen::Vector2d&,
                  const std::set<int>& >(),
        D(QuasiStaticConstraint, QuasiStaticConstraint, 2))
    .def("setActive", &QuasiStaticConstraint::setActive,
        D(QuasiStaticConstraint, setActive))
    .def("bounds", &QuasiStaticConstraint::bounds,
        D(QuasiStaticConstraint, bounds))
    .def("setShrinkFactor", &QuasiStaticConstraint::setShrinkFactor,
        D(QuasiStaticConstraint, setShrinkFactor))
    .def("addContact",
         static_cast<void(QuasiStaticConstraint::*)(
             std::vector<int>, const Eigen::Matrix3Xd&)>(
                 &QuasiStaticConstraint::addContact), D(QuasiStaticConstraints, addContact));

  py::class_<IKoptions>(m, "IKoptions", D(IKoptions))
    .def(py::init<RigidBodyTree<double>*>(), D(IKoptions, IKoptions))
    .def("setQ", &IKoptions::setQ, D(IKoptions, setQ))
    .def("getQ", &IKoptions::getQ, D(IKoptions, getQ))
    .def("setQa", &IKoptions::setQa, D(IKoptions, setQa))
    .def("getQa", &IKoptions::getQa, D(IKoptions, getQa))
    .def("setQv", &IKoptions::setQv, D(IKoptions, setQv))
    .def("getQv", &IKoptions::getQv, D(IKoptions, getQv))
    .def("setDebug", &IKoptions::setDebug, D(IKoptions, setDebug))
    .def("getDebug", &IKoptions::getDebug, D(IKoptions, getDebug))
    .def("setSequentialSeedFlag", &IKoptions::setSequentialSeedFlag, D(IKoptions, setSequentialSeedFlag))
    .def("getSequentialSeedFlag", &IKoptions::getSequentialSeedFlag, D(IKoptions, getSequentialSeedFlag))
    .def("setMajorOptimalityTolerance", &IKoptions::setMajorOptimalityTolerance, D(IKoptions, setMajorOptimalityTolerance))
    .def("getMajorOptimalityTolerance", &IKoptions::getMajorOptimalityTolerance, D(IKoptions, getMajorOptimalityTolerance))
    .def("setMajorFeasibilityTolerance",
         &IKoptions::setMajorFeasibilityTolerance, D(IKoptions, setMajorFeasibilityTolerance))
    .def("getMajorFeasibilityTolerance",
         &IKoptions::getMajorFeasibilityTolerance, D(IKoptions, getMajorFeasibilityTolerance))
    .def("setSuperbasicsLimit", &IKoptions::setSuperbasicsLimit, D(IKoptions, setSuperbasicsLimit))
    .def("getSuperbasicsLimit", &IKoptions::getSuperbasicsLimit, D(IKoptions, getSuperbasicsLimit))
    .def("setMajorIterationsLimit", &IKoptions::setMajorIterationsLimit, D(IKoptions, setMajorIterationsLimit))
    .def("getMajorIterationsLimit", &IKoptions::getMajorIterationsLimit, D(IKoptions, getMajorIterationsLimit))
    .def("setIterationsLimit", &IKoptions::setIterationsLimit, D(IKoptions, setIterationsLimit))
    .def("getIterationsLimit", &IKoptions::getIterationsLimit, D(IKoptions, getIterationsLimit))
    .def("setFixInitialState", &IKoptions::setFixInitialState, D(IKoptions, setFixInitialState))
    .def("getFixInitialState", &IKoptions::getFixInitialState, D(IKoptions, getFixInitialState))
    .def("setq0", &IKoptions::setq0, D(IKoptions, setq0))
    .def("getq0", &IKoptions::getq0, D(IKoptions, getq0))
    .def("setqd0", &IKoptions::setqd0, D(IKoptions, setqd0))
    .def("getqd0", &IKoptions::getqd0, D(IKoptions, getqd0))
    .def("setqdf", &IKoptions::setqdf, D(IKoptions, setqdf))
    .def("getqdf", &IKoptions::getqdf, D(IKoptions, getqdf))
    .def("setAdditionaltSamples", &IKoptions::setAdditionaltSamples, D(IKoptions, setAdditionaltSamples))
    .def("getAdditionaltSamples", &IKoptions::getAdditionaltSamples, D(IKoptions, getAdditionaltSamples));

  m.def("InverseKin",
        static_cast<IKResults(*)(
            RigidBodyTree<double>*,
            const Eigen::VectorXd&,
            const Eigen::VectorXd&,
            const std::vector<RigidBodyConstraint*>&,
            const IKoptions&)>(
                &inverseKinSimple), D(IKoptions, InverseKin));

  m.def("InverseKinPointwise",
        static_cast<IKResults(*)(
            RigidBodyTree<double>*,
            const Eigen::VectorXd&,
            const Eigen::MatrixXd&,
            const Eigen::MatrixXd&,
            const std::vector<RigidBodyConstraint*>&,
            const IKoptions&)>(
                &inverseKinPointwiseSimple), D(IKoptions, InverseKinPointwise));

  m.def("InverseKinTraj", &inverseKinTrajSimple, D(IKoptions, InverseKinTraj));

  py::class_<IKResults>(m, "IKResults", D(IKresults))
    .def_readonly("q_sol", &IKResults::q_sol, D(IKresults, q_sol))
    .def_readonly("info", &IKResults::info, D(IKresults, info))
    .def_readonly("infeasible_constraints", &IKResults::infeasible_constraints, D(IKresults, infeasible_constraints));
}

}  // namespace pydrake
}  // namespace drake
