"""List the robot hardware this container can see under /dev.

    python3 04_list_devices.py

Without `-v /dev:/dev` (or --device ...) a container has a tiny, fake /dev with
just null, zero, random, tty, shm, etc. With it, cameras and CAN/serial adapters
appear -- including udev symlinks like /dev/can_follower_l.
"""

import glob
import os

GROUPS = {
    "cameras (V4L2)": ["/dev/video*"],
    "CAN adapters (udev aliases)": ["/dev/can_*"],
    "serial / USB-CAN": ["/dev/ttyUSB*", "/dev/ttyACM*"],
    "raw USB bus": ["/dev/bus/usb/*/*"],
    "NVIDIA GPU": ["/dev/nvidia*"],
}


def describe(path):
    if os.path.islink(path):
        return f"{path} -> {os.path.realpath(path)}"
    return path


def main():
    total = len(os.listdir("/dev"))
    print(f"/dev has {total} entries\n")

    for label, patterns in GROUPS.items():
        found = sorted(p for pattern in patterns for p in glob.glob(pattern))
        print(f"{label}: {len(found)}")
        for path in found:
            # Seeing the file isn't enough; the device cgroup must allow opening it.
            readable = os.access(path, os.R_OK | os.W_OK)
            print(f"    {describe(path)}{'' if readable else '  (no read/write access)'}")

    if total < 30:
        print("\nLooks like Docker's minimal /dev. Try: -v /dev:/dev --privileged")


if __name__ == "__main__":
    main()
