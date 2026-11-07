# robot_simulator package

This directory contains the source code of the Lightweight Robotics Simulator. For installation and operation, refer to the [user guide](../docs/user_guide.md).

## Start the simulator from this package

```bash
python -m robot_simulator.main --help
```

The script `run.sh` in the repository root does the same. It also sources ROS 2 and the virtual environment.

## Contents

| Directory | Contents |
|---|---|
| `core/` | Application loops, simulation manager, clock, configuration, command queue, snapshots |
| `physics/` | PyBullet engine (DIRECT mode), robot connection, collision filters, motor commands |
| `robot/` | URDF parser, mesh and package resolver, robot hierarchy, loader |
| `motors/` | Rotational motor and linear motor |
| `control/` | Differential-drive controller |
| `sensors/` | Camera, perfect IMU and 2D LiDAR |
| `rendering/` | OpenGL renderer, shaders, meshes, scene builder, sensor overlays |
| `ros2_bridge/` | ROS 2 bridge thread, publishers, subscribers, services, TF |
| `environment/` | Environment objects |
| `project/` | Save and load of project files |
| `ui/` | ImGui user interface |
| `utils/` | Rigid-body math |
| `examples/` | Example robots and projects, ros2_control emulator (`ros2_integration/`) |
| `tests/` | pytest tests |
| `config.json` | Default configuration |

## Rules for the code

- Each subsystem must work and be testable alone. The `physics` package must not use the rendering, ROS 2 or the user interface.
- Only the simulation thread uses PyBullet. Other threads send commands through the command queue.
- Use SI units.
- Write comments and documentation in ASD-STE100 Simplified Technical English.
