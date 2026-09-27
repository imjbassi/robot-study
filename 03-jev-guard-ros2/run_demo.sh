#!/usr/bin/env bash
# Build jev_guard, start it with the offline mock backend, start the fake
# workcell, and drop you into a shell to poke at it with the topic 02 commands.
#
#   ./run_demo.sh                      # clean pick-and-place runs
#   ./run_demo.sh --fault collision    # or stall | slip | singularity
#   ./run_demo.sh --docker [--fault X] # same thing inside ros:humble
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"

if [[ "${1:-}" == "--docker" ]]; then
  shift
  exec docker run -it --rm --network host --ipc host \
    -v "$HERE:/study:ro" ros:humble /study/run_demo.sh "$@"
fi

# ROS's setup.bash reads unset variables, so relax -u while sourcing it.
set +u
# shellcheck disable=SC1091
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
set -u

if ! ros2 pkg prefix vision_msgs >/dev/null 2>&1; then
  echo "installing vision_msgs ..."
  apt-get update -qq && apt-get install -y -qq "ros-${ROS_DISTRO}-vision-msgs" >/dev/null
fi

# Build out of tree so nothing gets written into this folder.
WS="${WS:-/tmp/jev_ws}"
mkdir -p "$WS/src"
rm -rf "$WS/src/jev-guard-ros2"
cp -r "$HERE/jev-guard-ros2" "$WS/src/"
(cd "$WS" && colcon build --packages-select jev_guard >/dev/null)
set +u
# shellcheck disable=SC1091
source "$WS/install/setup.bash"
set -u

ros2 launch jev_guard safety_monitor.launch.py use_mock:=true &
python3 "$HERE/fake_workcell.py" "$@" &
trap 'kill $(jobs -p) 2>/dev/null' EXIT

sleep 3
cat <<'EOF'

jev_guard (mock backend) and fake_workcell are running. Try:
  ros2 node info /jev_guard
  ros2 topic hz /jev_guard/status
  ros2 topic echo /jev_guard/status --field data
  ros2 topic echo /jev_guard/estop
  ros2 service call /jev_guard/reset std_srvs/srv/Trigger
Exit this shell to stop everything.

EOF
bash -i
