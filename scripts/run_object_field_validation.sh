#!/usr/bin/env bash
set -eo pipefail

PROFILE="${1:-petal}"
DURATION="${2:-0}"
case "$PROFILE" in
  petal|rotary) ;;
  *) echo "usage: $0 {petal|rotary} [duration_seconds]" >&2; exit 2 ;;
esac

REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
ISAAC_SIM_ROOT="${ISAAC_SIM_ROOT:-${HOME}/isaac-sim-5.1.0}"
set -u
export ROS_DISTRO="humble"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
ISAAC_ROS_LIB="$ISAAC_SIM_ROOT/exts/isaacsim.ros2.bridge/humble/lib"
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}:$ISAAC_ROS_LIB"

PYTHON_CODE='path="'"$REPO_ROOT"'/scripts/run_object_field_validation_isaacsim51.py"; code=open(path, encoding="utf-8").read(); exec(compile(code, path, "exec"), {"__file__": path, "__name__": "__main__"})'
exec "$ISAAC_SIM_ROOT/python.sh" -c "$PYTHON_CODE" --profile "$PROFILE" --duration "$DURATION"
