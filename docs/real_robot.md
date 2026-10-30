# Real FR3

The `franka_real_bringup` package runs the physical FR3 and Franka Hand with the
**same ROS interfaces as the simulation** ([interfaces](interfaces.md)), so a
student package written against the lab can run on hardware. Everything runs in
the real-robot Docker image on the lab PC; only a CPU performance service is
installed on the host. For sessions, follow the [TA checklist](ta_checklist.md).

> **Safety.** `real.launch.py` connects to the robot but leaves the arm
> uncommanded by default. With `arm:=active`, or after activating
> `fr3_arm_controller`, the arm is under torque control. Only then run student
> code, and only while a person watches the robot with the external
> activation/stop device in hand.

## Differences from simulation

| | Simulation | Real robot |
|---|---|---|
| Time | `/clock`, `use_sim_time:=true` | wall clock, `use_sim_time:=false`, no `/clock` |
| Arm controller | position trajectory controller | torque (effort) trajectory controller, same name and action |
| Gripper | trajectory controller on both fingers | `fr3_gripper_bridge` → `franka_gripper_node`, same action |
| MoveIt speed | joint limits from the URDF | velocity caps plus 10 % default scaling (`config/joint_limits.yaml`) |
| Arm goals | executed as sent | checked by `trajectory_guard` first; invalid goals are rejected with a reason |
| Obstacles | simulated table | measured cell from `config/workspace.yaml`: 0.90 × 0.81 m table, wall 0.52 m to the robot's right |
| Camera | Gazebo ZED X Mini | ZED X Mini on a Jetson, streamed to this PC (see Camera) |

The container sets `LAB_MODE=real`; the student starter derives `use_sim_time`
from it. Code that hard-codes `use_sim_time=True` waits forever on hardware.

**Gripper.** `/fr3_gripper/follow_joint_trajectory` uses the final point's
finger positions; their sum is the total opening (0–0.08 m). A goal that
names only `fr3_finger_joint1`, as MoveIt's `hand` group sends
(`fr3_finger_joint2` mimics it), counts that finger twice. Opening goals
become `franka_gripper` moves. Closing goals become grasps that keep holding
with `grasp_force` (20 N by default, `config/gripper_bridge.yaml`). As in
simulation, the goal succeeds wherever the fingers stop: read the measured
opening on `/joint_states` to tell a grasped object from an empty close. The
hand moves at its own speed; trajectory timing is ignored. The native actions
remain available:
`/franka_gripper/{homing,move,grasp,gripper_action}` and `/franka_gripper/stop`.

**Camera.** The ZED X Mini is connected over GMSL2 to a Jetson, which streams
it with the ZED SDK (UDP port 30000) over its USB network link
(`192.168.55.1`; this PC is `192.168.55.100`). Start the stream before the
receiver: log in to the Jetson (password `jetson2025`), run the ZED sender
there (second line) and keep that terminal open:

```bash
ssh jetson@192.168.55.1
python "/usr/local/zed/samples/camera streaming/multi_sender/python/streaming_senders.py"
```

The `franka_lab_zed` container
(`docker/zed/`, started with `./scripts/run.sh --real --zed`) receives the
stream with the ZED SDK 5.4 and ZED ROS 2 wrapper, computes depth on this PC's
GPU (`NEURAL_LIGHT`) and publishes the simulation's topics: `/zed_x_mini/rgbd/{image,
camera_info,depth_image,points}` and `/zed_x_mini/right/{image,camera_info}`
at the camera's 30 Hz; the simulation uses 15 Hz. Each colour or depth image is
about 8 MB, so slow subscribers (for example in Python) receive fewer; lower
`general.pub_frame_rate` in `docker/zed/zed_stream.yaml`, or live with
`ros2 param set /zed_x_mini/zed_node general.pub_frame_rate 15.0`. The
resolution comes from the Jetson's stream (currently 1920 × 1080, the
simulation uses 1920 × 1200); images are `bgra8`, depth `32FC1` in metres.
The stream carries no IMU data, so `/zed_x_mini/imu` stays silent. Headers name
the wrapper's frames
(`zed_x_mini_left_camera_frame_optical`, `zed_x_mini_left_camera_frame` for the
point cloud, `zed_x_mini_imu_link`); `urdf/zed_frames.xacro` places them at
the camera's nominal optical centres until a hand-eye calibration replaces
them. Messages carry this PC's receive time, because the Jetson's clock is not
synchronised with it. The first start optimises the depth model for the GPU,
which takes a few minutes; the result is kept in the `zed_resources` volume.
Another stream address or port: set `ZED_STREAM_ADDRESS` or `ZED_STREAM_PORT`
before `run.sh`. The receiver's ZED SDK should match the Jetson's version.
If its log repeats "CAMERA NOT INITIALIZED", the Jetson still holds a previous
session (it then stops sending after a few seconds): `docker stop
franka_lab_zed`, wait about 15 s, then `docker start franka_lab_zed`.

**Additional robot interfaces.**

| Interface | Type | Purpose |
|---|---|---|
| `/action_server/error_recovery` | `franka_msgs/action/ErrorRecovery` | Clear a reflex/error after a stop |
| `/franka_robot_state_broadcaster/*` | various | Measured EE pose, external wrench, full robot state |
| `/service_server/set_load`, `.../set_full_collision_behavior`, … | `franka_msgs/srv/*` | Payload and collision thresholds |

The broadcaster's base-frame poses and wrenches (`current_pose`, `o_t_ee`,
`o_f_ext_hat_k`, …) are relative to `fr3_link0`, but their header names the
URDF root `world`, which is 0.8 m lower. Use TF (`world` → `fr3_hand_tcp`) for
poses in the lab's world frame.

**Trajectory guard.** `/fr3_arm_controller/follow_joint_trajectory` is served
by `trajectory_guard`, which checks every arm goal, from MoveIt or direct,
before the controller sees it. A goal is aborted with `INVALID_GOAL` or
`INVALID_JOINTS` and the reason in `error_string` unless it names exactly the 7
arm joints, contains only finite numbers and increasing times, stays within the
joint position limits, starts at the current position (a point at time 0 may be
at most 0.02 rad away), and no segment is faster on average than the MoveIt
velocity caps (1.0 rad/s for joints 1–4, 1.5 rad/s for 5–7). Interpolation can
briefly reach about twice the average, still below the FR3's own limits.
Accelerations and collisions are not checked: direct goals still bypass MoveIt's
collision checking, so send only slow, smooth trajectories. The controller's own
inputs (`follow_joint_trajectory`, `joint_trajectory`, `speed_scaling_input`)
are moved below `/fr3_arm_controller/_unchecked/`; do not use them. As a further
safety net the controller aborts a goal and holds the current pose when any
joint is more than 0.1 rad from its desired position (`constraints` in
`config/controllers.yaml`).

## One-time setup

1. **Docker access** for the account that runs the lab:
   `sudo usermod -aG docker $USER`, then log out and back in.
2. **Network.** FCI is on the Control's shop-floor network at `10.10.10.10`. The USB Ethernet
   adapter on this PC (`enx00e04c367450`) uses the static NetworkManager profile
   `franka-fci` (`10.10.10.50/24`, never the default route). Check with
   `ping 10.10.10.10`. To recreate it on another adapter:
   `nmcli connection add type ethernet con-name franka-fci ifname <iface> ipv4.method manual ipv4.addresses 10.10.10.50/24 ipv4.never-default yes ipv6.method link-local`.
3. **Desk** (`https://10.10.10.10`): unlock the joints, switch to Execution
   mode and activate FCI (System menu); repeat after every robot restart. The
   end-effector must be the **Franka Hand** (0.73 kg); otherwise the hand's own
   FCI server stays off. The camera and mount (0.2175 kg) are set by the
   bringup (see Start).
4. **libfranka version.** libfranka must be compatible with the robot's system
   version (Desk dashboard, "Control"). This lab's FR3 runs system **5.9.0**,
   which libfranka ≥ 0.18.0 requires. The image installs libfranka 0.20.5; the
   vendored franka_ros2 3.3.0 driver requires libfranka ≥ 0.20.4. After a robot
   system update, check the libfranka changelog and rebuild with
   `LIBFRANKA_VERSION=<version> ./scripts/build.sh --real`.
5. **CPU performance mode.** The 1 kHz control loop needs the CPU at full
   speed; power-profiles-daemon resets it to "balanced"/powersave at every
   boot. `sudo ./scripts/install_cpu_performance.sh` installs a service that
   selects the performance profile and governor at boot, and applies it now.

## Start

```bash
./scripts/build.sh --real
./scripts/run.sh --real          # add --nvidia on NVIDIA machines
```

Inside the container, bring the system up:

```bash
ros2 launch franka_real_bringup real.launch.py arm:=active
```

Then, in another container terminal (`docker compose exec lab bash`), run the
read-only check. It also reports the hardware, the arm controller state and,
on the robot, its mode and current errors:

```bash
ros2 run franka_real_bringup check_real_setup.py
```

Stage 3 can also be entered from a running stage 2, and left again, without
reconnecting (the arm holds its pose under torque control while active):

```bash
ros2 control switch_controllers --activate fr3_arm_controller
ros2 control switch_controllers --deactivate fr3_arm_controller
```

**Payload and collision thresholds.** Every bringup first sets the camera and
mount load and the collision reflex thresholds from
`config/robot_settings.yaml`. The arm controller is activated only afterwards,
because the robot rejects these settings while it is controlled. If a setting
fails, the bringup stops with the arm still inactive. The load (0.2175 kg,
centre of mass in the flange frame `fr3_link8`) is computed from the URDF; the
thresholds are franka_ros's defaults, and lower values stop the robot on
lighter contact. At rest, `/franka_robot_state_broadcaster/external_joint_torques`
stays within about 1.3 Nm on every joint.

**First motion.** `arm_motion_test.py` moves one joint by a small angle and
back (default: joint 7 by 0.1 rad at 0.1 rad/s; at most 0.3 rad and 0.2 rad/s).
Without `--execute` it only prints the goal. With `--execute` it states whether
it talks to mock hardware or the real robot and waits for you to type `move`;
Ctrl+C cancels the goal. Try it in stage 1 first:

```bash
ros2 run franka_real_bringup arm_motion_test.py                  # dry run
ros2 run franka_real_bringup arm_motion_test.py --execute        # sends the goal
ros2 run franka_real_bringup arm_motion_test.py --joint 4 --delta -0.1 --execute
```

Launch options: `robot_ip:=10.10.10.10`, `load_gripper:=false`, `rviz:=false`.
After powering on the hand, home it once (the fingers move):
`ros2 action send_goal /franka_gripper/homing franka_msgs/action/Homing {}`.

If the robot stops with a reflex (for example after a collision), check the
scene, then clear it with
`ros2 action send_goal /action_server/error_recovery franka_msgs/action/ErrorRecovery {}`.
This in-place recovery is not verified in this lab yet; if the arm stays
unresponsive, stop the bringup, release the stop device and launch again.
If the driver cannot connect (for example "Connection to FCI refused": activate
FCI in Desk), the bringup stops by itself: at once with the gripper, after about
a minute with `load_gripper:=false`. Fix the cause and launch again. The hand
has its own FCI server: if only `franka_gripper_node` reports the refusal, check
that Desk's end-effector is the Franka Hand and the hand is connected, or start
with `load_gripper:=false`.
