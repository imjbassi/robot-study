# 01 — Docker for ROS 2 Robots

A normal `docker run` gives you an isolated box: its own network, its own IPC
namespace, no GPU, no hardware, and a restricted kernel. That is great for web
apps and terrible for a robot. A typical robot container is started like this:

```bash
docker run -it --rm \
  --network host \
  --ipc host \
  --gpus all \
  -v /dev:/dev \
  --privileged \
  ros:humble
```

Each flag removes one layer of isolation. The demos in [`demos/`](demos/) let you
see what breaks without each one.

---

## `--network host` — (crucial) DDS discovery

ROS 2 sits on top of **DDS** (Data Distribution Service). DDS nodes find each other
with the **SPDP** discovery protocol, which sends **UDP multicast** packets to
`239.255.0.1` on port `7400 + 250 * domain_id` (so `7400` for `ROS_DOMAIN_ID=0`).

Docker's default **bridge** network puts the container behind NAT on a private
subnet (`172.17.0.0/16`). Multicast does not cross that bridge, so:

- nodes inside the container can see each other,
- but they are **invisible** to nodes on the robot's physical hardware / LAN, and vice versa.

`--network host` makes the container use the host's network stack directly — same
interfaces, same IPs, same multicast groups — so discovery just works.

**Demo:** [`demos/01_multicast_discovery.py`](demos/01_multicast_discovery.py) is a
tiny imitation of SPDP.

```bash
# Terminal 1 (host)
python3 demos/01_multicast_discovery.py listen

# Terminal 2 — WITH host networking: the host listener sees it
docker run --rm --network host -v "$PWD/demos:/demos" python:3.11-slim \
  python /demos/01_multicast_discovery.py announce --name container-node

# Terminal 2 — WITHOUT host networking: silence on the host
docker run --rm -v "$PWD/demos:/demos" python:3.11-slim \
  python /demos/01_multicast_discovery.py announce --name container-node
```

---

## `--ipc host` — zero-copy shared memory

Ties back to IPC: processes can pass data by writing it into a **shared memory**
segment (on Linux these live in `/dev/shm`) and handing over just a name, instead
of serializing and copying every byte. For 30 fps of 1080p RGB (~6 MB per frame)
that's the difference between ~180 MB/s of memcpy and basically nothing.

Each container gets its **own IPC namespace** (and its own private `/dev/shm`) by
default. So a camera-driver container and an AI-policy container can't see each
other's segments. With `--ipc host` on both, they share the host's IPC namespace
and the camera container can hand frames to the policy container zero-copy.

(ROS 2 does this itself with Iceoryx / Fast DDS shared-memory transport / "loaned
messages" — this is the same idea, stripped down.)

**Demo:** [`demos/02_shared_memory_frames.py`](demos/02_shared_memory_frames.py)
has a `camera` (writer) and a `policy` (reader).

```bash
# Container A: fake camera driver
docker run --rm --ipc host -v "$PWD/demos:/demos" python:3.11-slim \
  python /demos/02_shared_memory_frames.py camera

# Container B: fake policy — sees the frames
docker run --rm --ipc host -v "$PWD/demos:/demos" python:3.11-slim \
  python /demos/02_shared_memory_frames.py policy

# Drop --ipc host from either one and the policy can't find the segment.
```

Alternative: `--ipc container:<camera_container>` shares one container's namespace
with another without exposing the whole host. See
[`docker-compose.yml`](docker-compose.yml).

---

## `--gpus all` — the NVIDIA GPU

Containers don't see GPUs by default. With the **NVIDIA Container Toolkit**
installed on the host, `--gpus all` injects the GPU device nodes
(`/dev/nvidia*`) and the matching driver libraries into the container, so CUDA —
and therefore **PyTorch / JAX** — can run the VLA (vision-language-action) policy.

You can also pick specific GPUs: `--gpus '"device=0"'`.

**Demo:** [`demos/03_gpu_check.py`](demos/03_gpu_check.py)

```bash
docker run --rm --gpus all -v "$PWD/demos:/demos" pytorch/pytorch \
  python /demos/03_gpu_check.py
```

---

## `-v /dev:/dev` — the hardware

Linux exposes hardware as files under `/dev`. Bind-mounting the host's whole
`/dev` tree means the container sees everything the host sees, e.g.:

- `/dev/video0`, `/dev/video2` … — the cameras (V4L2)
- `/dev/can_follower_l` — a USB-CAN adapter, given a **stable name** by a udev rule
  (see [`99-robot-can.rules`](99-robot-can.rules)) instead of whatever
  `/dev/ttyACM*` number it got at boot
- `/dev/ttyUSB*`, `/dev/ttyACM*` — serial motor controllers

Mounting all of `/dev` (rather than `--device /dev/video0`) also means devices that
are **hot-plugged after** the container starts still show up.

Note: a mounted device file isn't enough by itself; the cgroup must allow access to
it too. That's one of the things `--privileged` does (or use
`--device-cgroup-rule`).

**Demo:** [`demos/04_list_devices.py`](demos/04_list_devices.py)

```bash
docker run --rm -v "$PWD/demos:/demos" python:3.11-slim python /demos/04_list_devices.py
docker run --rm -v /dev:/dev --privileged -v "$PWD/demos:/demos" python:3.11-slim \
  python /demos/04_list_devices.py
```

---

## `--privileged` — lift kernel restrictions

By default Docker drops most Linux **capabilities**, applies a **seccomp** filter,
and uses a device **cgroup** allow-list. `--privileged` turns all of that off:

- full access to every device (low-level USB, CAN, GPIO)
- `CAP_SYS_NICE` → can set **real-time scheduling** (`SCHED_FIFO`) for the control
  loop so it isn't preempted by, say, the logger
- `CAP_NET_ADMIN` → can bring up CAN interfaces (`ip link set can0 up type can bitrate 1000000`)

**Demo:** [`demos/05_realtime_priority.py`](demos/05_realtime_priority.py)

```bash
docker run --rm -v "$PWD/demos:/demos" python:3.11-slim \
  python /demos/05_realtime_priority.py          # fails: Operation not permitted
docker run --rm --privileged -v "$PWD/demos:/demos" python:3.11-slim \
  python /demos/05_realtime_priority.py          # works
```

**Trade-off:** a privileged container is basically root on the host. Fine on a
dedicated robot computer; for anything shared, prefer the narrower version:

```bash
--cap-add SYS_NICE --ulimit rtprio=99 \
--cap-add NET_ADMIN \
--device /dev/can_follower_l --device /dev/video0
```

---

## Files

| File | What |
|------|------|
| [`run_robot_container.sh`](run_robot_container.sh) | The full `docker run` command, commented flag by flag |
| [`docker-compose.yml`](docker-compose.yml) | Camera driver + policy as two services sharing IPC |
| [`99-robot-can.rules`](99-robot-can.rules) | Example udev rule that creates `/dev/can_follower_l` |
| [`demos/`](demos/) | One small stdlib-only Python script per flag |

## Check yourself

1. Why can two ROS 2 nodes in the *same* bridged container talk, but not to the robot?
2. What's in `/dev/shm`, and why does each container normally have its own?
3. What does the NVIDIA Container Toolkit actually add to the container?
4. Why mount all of `/dev` instead of `--device /dev/video0`?
5. Which capability do you need for `SCHED_FIFO`, and what's the least-privilege alternative to `--privileged`?
