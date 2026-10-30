# Third-party notices

## Franka robot description and configuration

`ros2_ws/src/franka_description/` is an FR3-only subset of Franka Robotics'
`franka_description` 2.7.0. The upstream license and notice are retained there.
Other robot variants, hardware-control integration and vendor demos are omitted;
the lab adds camera collision exclusions to the FR3 semantic description and
a micrometre of numerical tolerance at the simulated finger end stops.
Source: https://github.com/frankarobotics/franka_description (Apache-2.0).

The MoveIt controller, kinematics and joint-limit YAML files are derived from
`franka_ros2/franka_fr3_moveit_config`. Its Apache-2.0 license is retained in
`licenses/franka_ros2-LICENSE`.
Source: https://github.com/frankarobotics/franka_ros2.

`docker/franka_ros2/` holds unchanged copies of the `franka_hardware`,
`franka_gripper`, `franka_msgs`, `franka_robot_state_broadcaster` and
`franka_semantic_components` packages of `franka_ros2` 3.3.0, with its LICENSE
and NOTICE. The real-robot Docker image builds them as an underlay together with
the released `libfranka` Debian package (Apache-2.0).
Source: https://github.com/frankarobotics/franka_ros2.

## ZED X Mini and supplied bracket

`ros2_ws/src/franka_sim_bringup/meshes/zed_x_mini.stl` is an unchanged copy of
`meshes/zedxm.stl` from Stereolabs’ `zed-ros2-description`, commit
`22468247c1653d08a1e213809f3e9ce76291487d` (Apache-2.0). The license is retained
in `licenses/stereolabs-zed-description-LICENSE`. The 50 mm baseline and
nominal optical centre follow its `urdf/zed_macro.urdf.xacro`.
Source: https://github.com/stereolabs/zed-ros2-description/tree/22468247c1653d08a1e213809f3e9ce76291487d.

The mesh is in metres, with its origin at the bottom tripod screw. Simulated
projection centres use x=15.6 mm instead of upstream x=-5 mm to sit just ahead
of its opaque lens windows (x=15.487 mm). This rendering approximation is
included consistently in TF and does not claim hardware extrinsic calibration.

The 150 g mass, 2.2 mm wide lens and 0.1–8 m depth range follow Stereolabs’
[product information](https://www.stereolabs.com/en-ca/store/products/zed-x-mini-stereo-camera).
The nominal rectified HD1200 calibration (1920 × 1200, fx = fy = 737 px) follows
[Stereolabs’ focal-length and FOV reference](https://support.stereolabs.com/articles/8809264540-what-is-the-camera-focal-length-and-field-of-view).
Per-device factory calibration can differ. No ZED SDK or wrapper is bundled.

## FurnitureBench drawer meshes

`ros2_ws/src/franka_sim_bringup/models/drawer/` contains assets derived from
FurnitureBench's drawer model and AprilTag images:
https://github.com/clvrai/furniture-bench (MIT, copyright 2023 CLVR lab @ KAIST).

The body meshes are repaired and the knobs replaced with centred T-knobs. The
cabinet is trimmed to its original tray floor to form a 7.5 mm closed roof.
The remaining bodies retain their source geometry at 2x scale. Tag 52 is moved
to the front-right roof corner. The articulated SDF and collision primitives
are adaptations for this simulation. Original source copies and print/authoring
tools are retained in the instructor version, not this student distribution.

### MIT License

```
Copyright (c) 2023 Cognitive Learning for Vision and Robotics (CLVR) lab @ KAIST

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```
