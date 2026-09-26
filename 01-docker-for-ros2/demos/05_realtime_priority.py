"""Try to give this process real-time (SCHED_FIFO) priority, like a robot control loop.

    python3 05_realtime_priority.py

A 1 kHz motor control loop must not get preempted by a logger or a web UI.
SCHED_FIFO threads always run before normal (SCHED_OTHER) ones. Setting it needs
CAP_SYS_NICE, which Docker drops by default -- so this fails in a normal
container and succeeds with --privileged (or --cap-add SYS_NICE --ulimit rtprio=99).
"""

import os
import time

PRIORITY = 80  # 1..99, higher runs first


def capabilities():
    """Return the effective capability bitmask of this process (Linux only)."""
    with open("/proc/self/status") as f:
        for line in f:
            if line.startswith("CapEff:"):
                return int(line.split()[1], 16)
    return 0


def jitter_test(period_s=0.001, iterations=2000):
    """Run a fake 1 kHz loop and report how late each wakeup was."""
    lateness = []
    next_tick = time.perf_counter()
    for _ in range(iterations):
        next_tick += period_s
        time.sleep(max(0.0, next_tick - time.perf_counter()))
        lateness.append(time.perf_counter() - next_tick)
    lateness.sort()
    us = lambda s: s * 1e6
    print(f"    1 kHz loop lateness: median {us(lateness[len(lateness) // 2]):.0f} us, "
          f"p99 {us(lateness[int(len(lateness) * 0.99)]):.0f} us, max {us(lateness[-1]):.0f} us")


def main():
    CAP_SYS_NICE = 23
    caps = capabilities()
    print(f"effective capabilities: {caps:#x}")
    print(f"CAP_SYS_NICE: {'yes' if caps & (1 << CAP_SYS_NICE) else 'no'}")

    print("\nnormal scheduling (SCHED_OTHER):")
    jitter_test()

    try:
        os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(PRIORITY))
    except PermissionError as e:
        print(f"\nSCHED_FIFO priority {PRIORITY}: FAILED ({e})")
        print("Run the container with --privileged, or --cap-add SYS_NICE --ulimit rtprio=99")
        return

    print(f"\nreal-time scheduling (SCHED_FIFO, priority {PRIORITY}):")
    jitter_test()
    print("\nThe difference is biggest when the machine is busy -- try it with a stress test running.")


if __name__ == "__main__":
    main()
