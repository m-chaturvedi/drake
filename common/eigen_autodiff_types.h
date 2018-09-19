#pragma once

// Redundant #define guards for the benefit of @pybind11//:mkdoc.py.
#ifndef DRAKE_COMMON_EIGEN_AUTODIFF_TYPES_H_
#define DRAKE_COMMON_EIGEN_AUTODIFF_TYPES_H_

/// @file
/// This file contains abbreviated definitions for certain uses of
/// AutoDiffScalar that are commonly used in Drake.
/// @see also eigen_types.h

#ifndef DRAKE_COMMON_AUTODIFF_HEADER
// TODO(soonho-tri): Change to #error.
#warning Do not directly include this file. Include "drake/common/autodiff.h".

// Ensure that this header is self-contained for the benefit of
// @pybind11//:mkdoc.py.
#include <Eigen/Core>
#include <unsupported/Eigen/AutoDiff>

#endif  // DRAKE_COMMON_AUTODIFF_HEADER

#include <type_traits>

#include <Eigen/Dense>

#include "drake/common/eigen_types.h"

namespace drake {

/// An autodiff variable with a dynamic number of partials.
using AutoDiffXd = Eigen::AutoDiffScalar<Eigen::VectorXd>;

// TODO(hongkai-dai): Recursive template to get arbitrary gradient order.

/// An autodiff variable with `num_vars` partials.
template <int num_vars>
using AutoDiffd = Eigen::AutoDiffScalar<Eigen::Matrix<double, num_vars, 1> >;

/// A vector of `rows` autodiff variables, each with `num_vars` partials.
template <int num_vars, int rows>
using AutoDiffVecd = Eigen::Matrix<AutoDiffd<num_vars>, rows, 1>;

/// A dynamic-sized vector of autodiff variables, each with a dynamic-sized
/// vector of partials.
typedef AutoDiffVecd<Eigen::Dynamic, Eigen::Dynamic> AutoDiffVecXd;

}  // namespace drake

#endif  // DRAKE_COMMON_EIGEN_AUTODIFF_TYPES_H_
