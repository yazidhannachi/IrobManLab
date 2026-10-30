# Optional student workspace

The simulation does not depend on this package. Delete it, empty the modules,
or replace it with your team's package without changing the environment.

- `perception.py`: turn camera observations into labeled poses and dimensions.
- `planning.py`: turn a goal and perceived scene into a trajectory.
- `execution.py`: command the arm/gripper and evaluate feedback.
- `task.py`: coordinate your pipeline. The supplied node only waits.

The function signatures are suggestions, not a required architecture. All
algorithm functions raise `NotImplementedError`; no solution is implemented.
Start the idle shell with `ros2 run student_manipulation task`.
