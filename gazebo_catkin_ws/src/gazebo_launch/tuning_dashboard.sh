#!/bin/bash
# 浏览器调参界面；不启动Gazebo、不自动发运动命令。
set -euo pipefail
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
WS_DIR=$(cd -- "$SCRIPT_DIR/../.." && pwd)
if [[ ${1:-} != --demo ]]; then
    if [[ ! -f "$WS_DIR/devel/setup.bash" ]]; then
        echo "请先编译工作空间，再启动调参台。" >&2
        exit 1
    fi
    set +u
    source "$WS_DIR/devel/setup.bash"
    set -u
fi
exec /usr/bin/python3 "$WS_DIR/src/asrobot/scripts/tuning_dashboard.py" "$@"
