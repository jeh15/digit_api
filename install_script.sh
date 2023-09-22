# Build Digit API wheel:
bazel build //:digitapi

# Install wheel via pip:
WHEEL=("bazel-bin/"*.whl)
pip install ${WHEEL}
