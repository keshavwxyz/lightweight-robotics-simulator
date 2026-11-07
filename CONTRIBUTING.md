# Contributing

Thank you for your interest in this project. This document tells you how to report a problem and how to send a change.

## Report a problem

Open an issue on GitHub. Give this data:

1. The version of the simulator (or the commit).
2. Your operating system, CPU architecture (x86_64 or ARM) and GPU.
3. Your ROS 2 distribution, if you use ROS 2.
4. The command that you used.
5. What you expected and what occurred.
6. The console output. If the simulator stops responding, send `kill -USR1 <pid>` and add the stack output.

If possible, attach a small URDF or project file that shows the problem.

## Send a change

1. Fork the repository and make a branch.
2. Install the development environment as the [user guide](docs/user_guide.md) tells you.
3. Make your change. Keep each change small and about one subject.
4. Add or change tests in `robot_simulator/tests/` for the change.
5. Run the tests:

   ```bash
   source /opt/ros/jazzy/setup.bash    # optional
   source .venv/bin/activate
   PYTHONPATH=$PWD:$PYTHONPATH python -m pytest robot_simulator/tests -q
   ```

6. Make sure that `python -m pyflakes robot_simulator` shows no new problems.
7. Open a pull request. Tell what the change does and why it is necessary.

## Rules

- Keep the subsystems separate. The physics must not use the rendering, ROS 2 or the user interface.
- Only the simulation thread uses PyBullet. Other threads use the command queue.
- Use SI units.
- The simulator supports URDF only, two motor types and three sensor types. Before you add a new type, open an issue to discuss it.
- Write comments and documentation in ASD-STE100 Simplified Technical English (Issue 9). Use short sentences, the active voice and one instruction in each sentence.

## License

When you send a change, you agree that your change is under the Apache License 2.0 of this project.
