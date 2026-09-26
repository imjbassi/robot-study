"""Pass fake camera frames between processes through shared memory (zero-copy).

    python3 02_shared_memory_frames.py camera   # writer: fills frames at 30 fps
    python3 02_shared_memory_frames.py policy   # reader: reads the latest frame

The camera creates a segment named `robot_camera` (you can see it as
/dev/shm/robot_camera). The policy attaches to it by name and reads the pixels
in place -- nothing is copied or serialized.

Shared memory is scoped to an IPC namespace. Two containers only see the same
/dev/shm if they both use --ipc host (or one joins the other's namespace).
"""

import argparse
import signal
import struct
import sys
import time
from multiprocessing import shared_memory

NAME = "robot_camera"
WIDTH, HEIGHT, CHANNELS = 640, 480, 3
FRAME_BYTES = WIDTH * HEIGHT * CHANNELS
# Header: frame counter (uint64) + timestamp (float64). Written after the pixels
# so a reader that sees a new counter also sees the matching frame.
HEADER = struct.Struct("Qd")
SIZE = HEADER.size + FRAME_BYTES


def attach(name):
    """Attach to an existing segment without taking ownership of it."""
    try:
        return shared_memory.SharedMemory(name=name, track=False)  # Python 3.13+
    except TypeError:
        shm = shared_memory.SharedMemory(name=name)
        # Before 3.13 an attaching process would delete the segment on exit.
        from multiprocessing import resource_tracker
        resource_tracker.unregister(shm._name, "shared_memory")
        return shm


def camera(fps):
    try:
        shm = shared_memory.SharedMemory(name=NAME, create=True, size=SIZE)
    except FileExistsError:
        # Left over from a previous run that was killed.
        shared_memory.SharedMemory(name=NAME).unlink()
        shm = shared_memory.SharedMemory(name=NAME, create=True, size=SIZE)

    print(f"camera: created /dev/shm/{NAME} ({SIZE / 1e6:.1f} MB), writing {fps} fps. Ctrl+C to stop.")
    pixels = shm.buf[HEADER.size:]
    frame_id = 0
    try:
        while True:
            frame_id += 1
            # "Capture": fill the frame with a single value so the reader can check it.
            value = frame_id % 256
            pixels[:] = bytes([value]) * FRAME_BYTES
            HEADER.pack_into(shm.buf, 0, frame_id, time.time())
            if frame_id % fps == 0:
                print(f"camera: frame {frame_id}")
            time.sleep(1 / fps)
    except KeyboardInterrupt:
        pass
    finally:
        del pixels
        shm.close()
        shm.unlink()
        print("camera: removed segment")


def policy(timeout):
    deadline = time.time() + timeout
    while True:
        try:
            shm = attach(NAME)
            break
        except FileNotFoundError:
            if time.time() > deadline:
                sys.exit(
                    f"policy: no /dev/shm/{NAME} after {timeout}s.\n"
                    "Is the camera running, and do both containers share an IPC namespace (--ipc host)?"
                )
            time.sleep(0.5)

    print(f"policy: attached to /dev/shm/{NAME}. Ctrl+C to stop.")
    pixels = shm.buf[HEADER.size:]
    last = 0
    try:
        while True:
            frame_id, stamp = HEADER.unpack_from(shm.buf, 0)
            if frame_id != last:
                last = frame_id
                latency_ms = (time.time() - stamp) * 1000
                # Read a pixel straight out of shared memory -- no copy of the frame.
                print(f"policy: frame {frame_id}, pixel[0]={pixels[0]}, age {latency_ms:.2f} ms")
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        del pixels
        shm.close()


def stop_on_sigterm(signum, frame):
    raise KeyboardInterrupt


def main():
    # `docker stop` sends SIGTERM; treat it like Ctrl+C so the segment gets cleaned up.
    signal.signal(signal.SIGTERM, stop_on_sigterm)
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("role", choices=["camera", "policy"])
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--timeout", type=float, default=10.0, help="policy: seconds to wait for the camera")
    args = parser.parse_args()

    if args.role == "camera":
        camera(args.fps)
    else:
        policy(args.timeout)


if __name__ == "__main__":
    main()
