# Lab interfaces

All distances are metres, angles radians and timestamps ROS simulation time.
Set `use_sim_time=true` in your nodes. Read camera calibration from CameraInfo;
do not hard-code image dimensions or intrinsics. On the real robot the same
interfaces run on wall-clock time; see [Real FR3](real_robot.md).

This branch uses a **ZED X Mini**. RGB-D and right images are 1920 × 1200 at 15 Hz. The stereo
baseline is 0.05 m, with nominal wide-lens rectified intrinsics fx = fy = 737,
cx = 960, cy = 600 (about 105° × 78°), and depth clipping at 0.1–8 m.
Gazebo supplies rendered depth with synthetic noise, not ZED SDK stereo
matching. No hardware-calibrated accuracy is implied.

| Input | ROS type | Meaning |
|---|---|---|
| `/clock` | `rosgraph_msgs/msg/Clock` | Simulation time |
| `/joint_states` | `sensor_msgs/msg/JointState` | Robot arm and finger states |
| `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | Robot and camera transforms |
| `/zed_x_mini/rgbd/image` | `sensor_msgs/msg/Image` | Left RGB image |
| `/zed_x_mini/rgbd/depth_image` | `sensor_msgs/msg/Image` | Depth in metres (`32FC1`) |
| `/zed_x_mini/rgbd/camera_info` | `sensor_msgs/msg/CameraInfo` | Left camera calibration |
| `/zed_x_mini/rgbd/points` | `sensor_msgs/msg/PointCloud2` | RGB-D point cloud from the depth image and camera info: 1920 × 1200, `x y z rgb`, NaN where depth is invalid |
| `/zed_x_mini/right/image`, `/zed_x_mini/right/camera_info` | Image / CameraInfo | Right camera |
| `/zed_x_mini/imu` | `sensor_msgs/msg/Imu` | Camera IMU |

Use sensor-data QoS (best effort, volatile) for image subscriptions. Check
message encodings and finite depth values. Pair observations by their timestamps
and transform estimates using TF at the observation time.

## Frames and geometry

- `world`: stationary frame; +z is up. The table top is at z = 0.8 m.
- `fr3_link0`: robot base, fixed to the table.
- `fr3_hand`: gripper palm, at −45° about the flange (`fr3_link8`) z axis, as
  the Franka Hand is mounted on the physical FR3.
- `fr3_hand_tcp`: MoveIt's arm tip. In this lab it **coincides with the palm**;
  it is not the fingertip contact point. Use the robot geometry/TF when defining
  a grasp frame. No task-specific grasp offsets are provided.
- `left_camera_link_optical`, `right_camera_link_optical`: camera optical frames,
  with +x right, +y down, +z forward. Use the frame in each message header.
- `zed_x_mini_mount`: original STL axes, with millimetres converted to metres.
  Its pose in `fr3_hand` is xyz = (-0.025, 0.040, 0.017) m, yaw = π.
- `camera_link`: manufacturer CAD reference at the bottom tripod screw.
  Its pose in `fr3_hand` is xyz = (-0.0906689, 0, 0.0028568) m,
  pitch = -50°. Left/right projection centres are at
  (0.0156, ±0.025, 0.016) m in `camera_link`, just ahead of the opaque CAD
  lens windows. This forward offset is a rendering approximation; upstream
  nominal optical centres are (-0.005, ±0.025, 0.016) m.
  The mount is confirmed fitted around the physical hand's base flange.
  The assembly pose is still inferred from CAD and requires hand-eye
  calibration. See [mount notes](../cad/zed_x_mini/README.md).

Both cameras publish matching K and P focal lengths. The left P translation is
zero; the right P translation is -fx × 0.05, for rectified stereo consumers.

The drawer markers use **AprilTag 36h11**, not an ArUco dictionary. Tag 52 is on
its top surface. Marker footprints are 52.5 mm square including the outer white
margin; the black square is 42 mm wide. The drawer model and robot description
provide known physical geometry; students estimate their current relative poses.

## Commands

| Interface | Type | Purpose |
|---|---|---|
| `/move_action` | `moveit_msgs/action/MoveGroup` | Plan / execute through MoveIt |
| `/execute_trajectory` | `moveit_msgs/action/ExecuteTrajectory` | Execute a planned trajectory |
| `/apply_planning_scene` | `moveit_msgs/srv/ApplyPlanningScene` | Add/update perceived obstacles and attached objects |
| `/get_planning_scene` | `moveit_msgs/srv/GetPlanningScene` | Inspect the planning scene |
| `/fr3_arm_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | Direct joint trajectory execution |
| `/fr3_gripper/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | Gripper trajectory execution |

The planning group is `fr3_arm`; joints are `fr3_joint1` through `fr3_joint7`.
The gripper controller commands **both** `fr3_finger_joint1` and
`fr3_finger_joint2`. Each travels 0–0.04 m; their sum is the total opening.
Direct controller commands do not perform MoveIt collision checking.

The robot's self-collision model and fixed work surface are configured. Object
and drawer obstacles, grasp frames, attachments, goals, retries and task state
belong to the student implementation. RViz can inspect the planning scene and
send manual planning requests; no course task is automated.
