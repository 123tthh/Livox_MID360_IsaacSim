#!/usr/bin/env bash
set -euo pipefail
mid360_profile="${1:-petal}"
mid360_duration="${2:-0}"
case "$mid360_profile" in
    petal|rotary) ;;
    *) echo "usage: $0 {petal|rotary} [duration_seconds] [--headless]" >&2; exit 2 ;;
esac
if [[ $# -gt 0 ]]; then shift; fi
if [[ $# -gt 0 ]]; then shift; fi
mid360_repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
exec bash "$mid360_repo/scripts/run_isaacsim62.sh" \
    "$mid360_repo/scripts/run_object_field_validation_isaacsim62.py" \
    --profile "$mid360_profile" --duration "$mid360_duration" "$@"
