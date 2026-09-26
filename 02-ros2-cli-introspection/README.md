# 02 — ROS 2 CLI Introspection

When a robot "does nothing", the first question is always: **is data flowing, and
who is talking to whom?** The `ros2` command-line tool answers that without writing
any code. These five commands cover most debugging sessions:

| Command | What it does |
|---|---|
| `ros2 topic list` | Lists all active data streams. |
| `ros2 topic hz /joint_states` | Measures the publish rate (Hertz) of a topic. |
| `ros2 topic echo /cmd_vel` | Prints the raw payload data to the terminal. |
| `ros2 node info /openpi_node` | Shows everything a node is connected to. |
| `ros2 wtf` / `ros2 doctor` | Runs a system-wide diagnostic check. |

## The practice system

[`nodes/`](nodes/) has two small `rclpy` nodes so you have something real to inspect:

```
 ┌──────────────┐   /joint_states  (50 Hz)    ┌───────────────┐
 │              │ ──────────────────────────▶ │               │
 │  fake_robot  │   /camera/image_raw (30 Hz) │  openpi_node  │
 │              │ ──────────────────────────▶ │  (the policy) │
 │              │ ◀────────────────────────── │               │
 └──────────────┘   /cmd_vel       (10 Hz)    └───────────────┘
```

- **`fake_robot.py`**: a 6-joint arm plus a camera. It moves when it gets `/cmd_vel`.
- **`openpi_node.py`**: stands in for a VLA policy. It reads observations and publishes actions, and has a `reset` service.

Start both, then get a shell to run commands in:

```bash
./start_demo.sh            # if ROS 2 is installed
./start_demo.sh --docker   # otherwise: runs in ros:humble with --network host (see topic 01)
```

All the output below is real output from this setup on ROS 2 Humble.

---

## `ros2 topic list`: what streams exist?

```
$ ros2 topic list -t
/camera/image_raw [sensor_msgs/msg/Image]
/cmd_vel [geometry_msgs/msg/Twist]
/joint_states [sensor_msgs/msg/JointState]
/parameter_events [rcl_interfaces/msg/ParameterEvent]
/rosout [rcl_interfaces/msg/Log]
```

- `-t` adds the message type. You need the type for `echo`, `pub` and writing code.
- `/parameter_events` and `/rosout` always exist; every node creates them.
- A topic appears as soon as **any** node creates a publisher **or** a subscriber. Being listed does not mean data is flowing. That's what `hz` is for.
- If a topic you expected is missing, check the spelling and namespace (`/robot1/joint_states` ≠ `/joint_states`), then check discovery (see the domain exercise below).

## `ros2 topic hz /joint_states`: is it alive, and how fast?

```
$ ros2 topic hz /joint_states
average rate: 50.003
	min: 0.019s max: 0.021s std dev: 0.00030s window: 103
```

- The **rate** tells you the driver is alive. A controller expecting 50 Hz that gets 12 Hz will behave badly.
- **min/max/std dev** show jitter. A large `max` means the publisher stalled, e.g. the CPU was busy or there's a GC pause. This is the reason topic 01 cares about real-time scheduling.
- No output at all means nothing is being received. Either nobody is publishing, or the QoS settings don't match (see below).
- Related: `ros2 topic bw /camera/image_raw` shows **bandwidth** (`278.44 KB/s ... Message size mean: 9.27 KB`). Use it to check whether uncompressed images are filling up your network.

## `ros2 topic echo /cmd_vel`: what is actually in the messages?

```
$ ros2 topic echo /cmd_vel --once
linear:
  x: -0.3
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0
---
```

Useful flags:

| Flag | Why |
|---|---|
| `--once` | Print one message and exit |
| `--field position` | Print just one field, e.g. `ros2 topic echo /joint_states --field position` |
| `--no-arr` | Don't dump big arrays. Image data becomes `'<sequence type: uint8, length: 9216>'` |
| `--qos-reliability best_effort` | Match a best-effort publisher so you can hear it |

The reverse is `ros2 topic pub`, which sends a message by hand. This is handy for
testing a robot without the policy running. **Careful on real hardware:**

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"
```

## `ros2 node info /openpi_node`: what is this node wired to?

```
$ ros2 node info /openpi_node
/openpi_node
  Subscribers:
    /camera/image_raw: sensor_msgs/msg/Image
    /joint_states: sensor_msgs/msg/JointState
  Publishers:
    /cmd_vel: geometry_msgs/msg/Twist
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
  Service Servers:
    /openpi_node/describe_parameters: rcl_interfaces/srv/DescribeParameters
    ...
    /openpi_node/reset: std_srvs/srv/Trigger
    ...
  Service Clients:

  Action Servers:

  Action Clients:
```

- This is the quickest way to find a **name mismatch**. If the policy subscribes to `/joint_state` but the robot publishes `/joint_states`, you'll see it here.
- The six `*_parameters` services come with every node. They're what `ros2 param` uses:
  ```
  $ ros2 param get /fake_robot joint_rate
  Double value is: 50.0
  ```
- You can call the node's own services from the CLI:
  ```
  $ ros2 service call /openpi_node/reset std_srvs/srv/Trigger
  response:
  std_srvs.srv.Trigger_Response(success=True, message='policy state reset')
  ```
- Related: `ros2 node list` lists all nodes, and `ros2 topic info /joint_states --verbose` shows who publishes and subscribes to a topic, including their **QoS**.

## `ros2 doctor` (alias `ros2 wtf`): whole-system checkup

```
$ ros2 doctor
All 3 checks passed

$ ros2 doctor --report
   NETWORK CONFIGURATION
...
device       : eth0
flags        : 4163<UP,RUNNING,BROADCAST,MULTICAST>
...
   QOS COMPATIBILITY LIST
topic [type]            : /joint_states [sensor_msgs/msg/JointState]
publisher node          : fake_robot
subscriber node         : openpi_node
compatibility status    : OK
...
```

- `ros2 wtf` is literally the same command, under a funnier name.
- The plain check covers the network, installed packages, platform and QoS. `--report` prints everything it found.
- Look for **`MULTICAST`** in the interface flags. Without multicast, DDS discovery can't work (topic 01).
- The **QoS compatibility list** is the most useful part. It catches the silent failure in exercise 1.
- In a container with no internet you'll see `UserWarning: Fail to call PackageCheck`. It tries to reach rosdistro online, and you can ignore it.

---

## Exercises: break it, then find the problem

### 1. Silent QoS mismatch

Start the robot publishing `/joint_states` as **best effort**. The policy subscribes as **reliable**, the default:

```bash
python3 nodes/fake_robot.py --ros-args -p best_effort:=true &
python3 nodes/openpi_node.py &
```

The robot is publishing, but the policy never acts. Diagnose it:

```
$ ros2 topic hz /cmd_vel
                                   <- nothing: the policy isn't producing actions

$ ros2 topic info /joint_states -v | grep -E "Node name|Reliability"
Node name: fake_robot
  Reliability: BEST_EFFORT
Node name: openpi_node
  Reliability: RELIABLE

$ ros2 doctor --report
compatibility status    : ERROR: Best effort publisher and reliable subscription;
```

**Rule:** a *reliable* subscriber can't receive from a *best-effort* publisher. The reverse works fine.
Camera and lidar drivers often publish best effort, so this happens a lot in practice. The fix is
to make the subscriber use `qos_profile_sensor_data`, as `openpi_node` already does for the camera.

### 2. Wrong `ROS_DOMAIN_ID`

```bash
python3 nodes/fake_robot.py &
ROS_DOMAIN_ID=1 python3 nodes/openpi_node.py &
```

```
$ ros2 node list                    # the CLI uses domain 0 by default
/fake_robot
$ ROS_DOMAIN_ID=1 ros2 node list
/openpi_node
```

Nodes in different domains can't see each other at all. Domains are separate
discovery ports, 7400 + 250 × domain, like in topic 01's multicast demo. This is how you stop two robots on the same Wi-Fi from
driving each other, and it's also a common reason for "my node is invisible". The CLI is a node
too, so it only sees its own domain.

### 3. Your turn

- Change the joint rate at runtime with `ros2 param set /fake_robot joint_rate 10.0`. Does `hz` change? Why not? (Hint: when is the timer created?)
- Send `/cmd_vel` by hand with `ros2 topic pub` while `openpi_node` is stopped, and watch `--field position` on `/joint_states`.
- Compare `ros2 topic bw` for the camera with the 640×480 frames from topic 01's shared-memory demo. How much bandwidth would 30 fps at 1080p need?

## Check yourself

1. A topic appears in `topic list` but `topic hz` prints nothing. Name two possible causes.
2. Which command shows a topic's QoS, and which shows QoS mismatches across the whole system?
3. How do you print only the `position` array from `/joint_states`?
4. What does `ros2 node info` show that `ros2 topic list` doesn't?
5. Why can't the CLI see a node running with `ROS_DOMAIN_ID=1`?
