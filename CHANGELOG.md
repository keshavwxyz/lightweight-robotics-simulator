# Changelog

All important changes to this project are in this file.

## 0.1.0

First public version.

### Added

- PyBullet physics in DIRECT mode with a fixed timestep.
- URDF import: fixed, revolute, continuous and prismatic joints, mimic joints, box, cylinder, sphere and mesh geometry, `package://` paths, warnings for unsupported items.
- Two motor types (rotational and linear) with position, velocity and effort control. Differential-drive controller.
- Three sensor types: RGB camera, perfect IMU and 2D LiDAR (360° or partial field of view).
- Custom OpenGL 3.3 renderer with textured meshes, sensor overlays and collision view.
- ImGui user interface: scene hierarchy, properties, environment editor, console, camera windows.
- ROS 2 bridge: joint states, TF, odometry, sensor topics, `cmd_vel`, joint commands, `/clock`, simulation services. Namespace, topic and TF settings for each robot.
- JSON project files.
- Headless mode.
- ros2_control controller emulator for existing projects (`robot_simulator/examples/ros2_integration/`).
- User guide and porting guide.
