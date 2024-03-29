workspace(name = "lowlevelapi")

# Setup Hermetic Python toolchain:
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

SHA="5868e73107a8e85d8f323806e60cad7283f34b32163ea6ff1020cf27abef6036"

VERSION="0.25.0"

http_archive(
    name = "rules_python",
    sha256 = SHA,
    strip_prefix = "rules_python-{}".format(VERSION),
    url = "https://github.com/bazelbuild/rules_python/releases/download/{}/rules_python-{}.tar.gz".format(VERSION,VERSION),
)

load("@rules_python//python:repositories.bzl", "py_repositories")

py_repositories()

load("@rules_python//python:repositories.bzl", "python_register_toolchains")

# Pybind11 currently does not support python 3.11:
python_register_toolchains(
    name = "python_3_10",
    python_version = "3.10",
)

load("@python_3_10//:defs.bzl", "interpreter")
load("@rules_python//python:pip.bzl", "pip_parse")

pip_parse(
  python_interpreter_target = interpreter,
)


# Pybind11:
http_archive(
  name = "pybind11_bazel",
  strip_prefix = "pybind11_bazel-2.11.1",
  sha256 = "2c466c9b3cca7852b47e0785003128984fcf0d5d61a1a2e4c5aceefd935ac220",
  urls = ["https://github.com/pybind/pybind11_bazel/releases/download/v2.11.1/pybind11_bazel-2.11.1.zip"],
)
http_archive(
  name = "pybind11",
  build_file = "@pybind11_bazel//:pybind11.BUILD",
  strip_prefix = "pybind11-2.11.1",
  sha256 = "d475978da0cdc2d43b73f30910786759d593a9d8ee05b1b6846d1eb16c6d2e0c",
  urls = ["https://github.com/pybind/pybind11/archive/refs/tags/v2.11.1.tar.gz"],
)

load("@pybind11_bazel//:python_configure.bzl", "python_configure")

python_configure(
  name = "local_config_python",
  python_interpreter_target = interpreter,
)

# ZSTD:
load("@bazel_tools//tools/build_defs/repo:git.bzl", "new_git_repository")

new_git_repository(
    name = "zstd",
    remote = "git@github.com:facebook/zstd.git",
    commit = "823b32ce430cd21706a5ffb5bd9e2e6297d919cc",
    build_file = "//third_party/zstd:BUILD.bazel",
)

# Eigen:
new_git_repository(
    name = "eigen",
    remote = "https://gitlab.com/libeigen/eigen.git",
    commit = "3147391d946bb4b6c68edd901f2add6ac1f31f8c",
    build_file = "//third_party/eigen:BUILD.bazel",
)
