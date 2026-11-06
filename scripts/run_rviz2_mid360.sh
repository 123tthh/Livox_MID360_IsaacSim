#!/usr/bin/env bash
set -eo pipefail

REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
PROFILE="${1:-petal}"
case "$PROFILE" in
  petal)
    RVIZ_CONFIG="$REPO_ROOT/config/mid360_object_field_petal_4s.rviz"
    ;;
  rotary)
    RVIZ_CONFIG="$REPO_ROOT/config/mid360_object_field_rotary_0p1s.rviz"
    ;;
  *)
    echo "usage: $0 [petal|rotary]" >&2
    exit 2
    ;;
esac
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
set -u
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
exec rviz2 -d "$RVIZ_CONFIG" --ros-args -r "__node:=mid360_${PROFILE}_rviz2"
