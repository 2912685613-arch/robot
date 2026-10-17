#!/usr/bin/env bash
set -u

# Resolve the workspace from this script instead of assuming the caller's cwd.
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd -- "$SCRIPT_DIR/../.." && pwd)"
source "$WS/devel/setup.bash"

# Default is the flat land world.  To use another world file while keeping
# the corrected Gazebo search paths, set WORLD_NAME, for example:
#   WORLD_NAME="$WS/src/gazebo_world/worlds/water_land_15_15_20_4.world" \
#     bash single_land.sh
WORLD_NAME="${WORLD_NAME:-$WS/src/gazebo_world/worlds/gazebo.world}"

if [[ ! -f "$WORLD_NAME" ]]; then
  echo "[single_land] world file not found: $WORLD_NAME" >&2
  exit 1
fi
if ! roslaunch --files gazebo_model gazebo_model.launch >/dev/null 2>&1; then
  echo "[single_land] gazebo_model/gazebo_model.launch is not available" >&2
  exit 1
fi

start_terminal() {
  local title="$1"
  shift
  dbus-launch gnome-terminal --title="$title" -- bash -lc "source '$WS/devel/setup.bash'; $*; exec bash"
}

# Keep separate terminals for compatibility with the existing workflow, but
# pass the selected world explicitly and do not rely on the caller's cwd.
start_terminal world "roslaunch --disable-title gazebo_world gazebo_world.launch world_name:='$WORLD_NAME'"
sleep 3
start_terminal model "roslaunch --disable-title gazebo_model gazebo_model.launch"
sleep 3
start_terminal control "roslaunch --disable-title gazebo_control gazebo_control.launch mode:=land"
