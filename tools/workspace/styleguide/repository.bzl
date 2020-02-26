# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def styleguide_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/styleguide",
        commit = "65fe06b2afb11fb45dc5853d87576c3d01c38070",
        sha256 = "57ec05822d92192711ce920f7f9f9516c06fbe2a436efa1d77b330062b535029",  # noqa
        build_file = "@drake//tools/workspace/styleguide:package.BUILD.bazel",
        mirrors = mirrors,
    )
