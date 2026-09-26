#!/usr/bin/env bash
# Start a ROS 2 container that can actually talk to a robot.
# Usage: ./run_robot_container.sh [image] [command...]
set -euo pipefail

IMAGE="${1:-ros:humble}"
shift || true

args=(
  -it --rm
  --name robot

  # DDS discovery uses UDP multicast (239.255.0.1:7400). The default bridge
  # network doesn't forward it, so share the host's network stack instead.
  --network host

  # Share the host's IPC namespace (/dev/shm, SysV IPC) so another container
  # (e.g. the camera driver) can hand us frames via shared memory, zero-copy.
  --ipc host

  # Give the container every host device node...
  -v /dev:/dev

  # ...and drop the capability/seccomp/cgroup restrictions so we can use them,
  # set SCHED_FIFO real-time priority, and bring up CAN interfaces.
  --privileged

  # Keep ROS 2 nodes in the same domain as the robot.
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
)

# Only ask for GPUs if the NVIDIA Container Toolkit is installed, otherwise
# `docker run` fails with "could not select device driver".
if command -v nvidia-ctk >/dev/null 2>&1; then
  # Pass the NVIDIA GPU(s) through so PyTorch/JAX can run the policy.
  args+=(--gpus all)
else
  echo "nvidia-ctk not found; starting without --gpus all" >&2
fi

exec docker run "${args[@]}" "$IMAGE" "$@"
