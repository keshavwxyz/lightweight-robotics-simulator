#!/usr/bin/env bash
# Start the simulator. If ROS 2 is installed and not sourced, this script sources it.
# The simulator also runs without ROS 2.
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -z "$ROS_DISTRO" ]; then
  for d in jazzy rolling humble; do
    if [ -f "/opt/ros/$d/setup.bash" ]; then source "/opt/ros/$d/setup.bash"; break; fi
  done
fi
if [ -f "$HERE/.venv/bin/activate" ]; then source "$HERE/.venv/bin/activate"; fi
export PYTHONPATH="$HERE${PYTHONPATH:+:$PYTHONPATH}"
exec python -m robot_simulator.main "$@"
