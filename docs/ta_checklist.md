# Real FR3 session checklist (TAs)

The arm only moves while a supervisor watches it with the external
activation/stop device in hand. Details: [Real FR3](real_robot.md).

## Once per PC

- Docker access, the `franka-fci` network profile and the built image
  (`./scripts/build.sh --real --zed`), as in [one-time setup](real_robot.md#one-time-setup).
- CPU performance mode at every boot: `sudo ./scripts/install_cpu_performance.sh`.

## Before the session

1. Power on the robot and wait until Desk (`https://10.10.10.10`) is ready.
2. In Desk: unlock the joints, switch to **Execution** mode, and
   **System → Activate FCI**. The end-effector must be the Franka Hand.
3. On the PC, check that every CPU line says `performance`:
   ```bash
   sort /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor | uniq -c
   ```
4. Start the camera stream on the Jetson: log in (password `jetson2025`), run
   the sender there (second line) and keep this terminal open during the session:
   ```bash
   ssh jetson@192.168.55.1
   python "/usr/local/zed/samples/camera streaming/multi_sender/python/streaming_senders.py"
   ```
   Then start the lab and camera containers: `./scripts/run.sh --real --nvidia --zed`.
5. In the container, start the bringup. The arm is **not** commanded yet:
   ```bash
   ros2 launch franka_real_bringup real.launch.py
   ```
   It stops by itself if it cannot reach the robot (activate FCI, retry).
6. In a second container terminal (`docker compose exec lab bash`), every line
   must say PASS, the camera topics included:
   ```bash
   ros2 run franka_real_bringup check_real_setup.py
   ```
7. activate the arm. It holds its pose under
   torque control and accepts goals.
   ```bash
   ros2 control switch_controllers --activate fr3_arm_controller
   ```

## During the session

- Someone holds the stop device whenever the arm is active.
- In RViz, keep Velocity/Accel. Scaling low (0.10 for first runs), press
  **Plan**, check the preview, then **Execute**.
- Arm goals that start away from the current pose, leave the joint limits or
  exceed the speed caps are rejected; the reason is in the result's
  `error_string` and in the bringup terminal.
- After the stop device was pressed or the robot stopped with a reflex: stop
  the bringup with Ctrl+C, check the scene, release the stop device and start
  again from step 5. (The in-place recovery procedure is not verified yet.)

## After the session

1. Deactivate the arm, then stop the bringup with Ctrl+C:
   ```bash
   ros2 control switch_controllers --deactivate fr3_arm_controller
   ```
2. Leave the container (`exit`); `docker compose down` stops it.
3. In Desk: lock the joints, then shut the robot down from Desk before
   switching off the power.
