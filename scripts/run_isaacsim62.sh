#!/usr/bin/env bash
# Invoke a script with the active 6.2 Python and the matching built-in Jazzy.
set -euo pipefail
mid360_python="${ISAAC_SIM_PYTHON:-python3}"
if [[ $# -lt 1 ]]; then
    echo "usage: $0 script.py [arguments...] (set ISAAC_SIM_PYTHON if needed)" >&2
    exit 2
fi
if [[ -n "${AMENT_PREFIX_PATH:-}" || -n "${ROS_DISTRO:-}" && "${ROS_DISTRO}" != jazzy ]]; then
    echo "Use a fresh terminal without a sourced system ROS workspace; main uses Sim 6.2 built-in Jazzy." >&2
    exit 1
fi
mid360_ros_lib="$("$mid360_python" - <<'PY'
from importlib.metadata import distribution
d = distribution('isaacsim')
if not d.version.startswith('6.2.'):
    raise SystemExit('main requires Isaac Sim 6.2; use branch isaac-sim-5.1 for 5.1')
p = d.locate_file('isaacsim/exts/isaacsim.ros2.core/jazzy/lib')
if not (p / 'libament_index_cpp.so').is_file():
    raise SystemExit('Missing Isaac Sim built-in Jazzy libraries: ' + str(p))
print(p)
PY
)"
export ROS_DISTRO=jazzy ROS_VERSION=2 ROS_PYTHON_VERSION=3
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export LD_LIBRARY_PATH="${mid360_ros_lib}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
exec "$mid360_python" "$@"
