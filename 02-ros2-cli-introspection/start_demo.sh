#!/usr/bin/env bash
# Start the fake robot and the policy in the background, then drop you into a
# shell where you can run the ros2 commands from the README.
#
#   ./start_demo.sh                 # on a machine with ROS 2 installed
#   ./start_demo.sh --docker        # inside a ros:humble container (uses topic 01's flags)
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"

if [[ "${1:-}" == "--docker" ]]; then
  exec docker run -it --rm --network host --ipc host \
    -v "$HERE:/study" -w /study ros:humble ./start_demo.sh
fi

# ROS's setup.bash reads unset variables, so relax -u while sourcing it.
set +u
# shellcheck disable=SC1091
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
set -u

python3 "$HERE/nodes/fake_robot.py" &
python3 "$HERE/nodes/openpi_node.py" &
trap 'kill $(jobs -p) 2>/dev/null' EXIT

sleep 2
echo
echo "fake_robot and openpi_node are running. Try:"
echo "  ros2 topic list"
echo "  ros2 topic hz /joint_states"
echo "  ros2 topic echo /cmd_vel"
echo "  ros2 node info /openpi_node"
echo "  ros2 doctor --report"
echo "Exit this shell to stop them."
echo
bash -i
