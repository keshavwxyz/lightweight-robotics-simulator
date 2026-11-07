# User guide

This guide tells you how to install, operate and configure the Lightweight Robotics Simulator.

## Summary

| Item | Description |
|---|---|
| Physics | PyBullet in DIRECT mode. The simulator does not use the PyBullet GUI. |
| Rendering | Custom OpenGL 3.3 core renderer. Dear ImGui user interface on GLFW. |
| Robot description | URDF only. |
| Motors | Two types: rotational and linear. |
| Sensors | Three types: RGB camera, perfect IMU and 2D LiDAR. |
| Middleware | ROS 2 (`rclpy`). The development used ROS 2 Jazzy. ROS 2 is optional. |

All values use SI units: metres, radians, seconds, kilograms, newtons and N·m.

## 1. Installation

These instructions are for Ubuntu 24.04.

1. Install ROS 2 Jazzy (optional). For the spawn, delete and state services, also install the simulation interfaces:

   ```bash
   sudo apt install ros-jazzy-simulation-interfaces
   ```

2. Make a Python virtual environment in the repository directory. The option `--system-site-packages` keeps `rclpy` from ROS 2 available:

   ```bash
   python3 -m venv --system-site-packages .venv
   source .venv/bin/activate
   pip install -r requirements.txt
   ```

3. Before you start the simulator with ROS 2, source ROS 2:

   ```bash
   source /opt/ros/jazzy/setup.bash
   ```

   The script `run.sh` does this step and step 2 for you.

## 2. Start the simulator

| Command | Result |
|---|---|
| `./run.sh` | Opens an empty scene. |
| `./run.sh --project <project.json> --play` | Opens a project and starts the simulation. |
| `./run.sh --urdf <robot.urdf>` | Imports a URDF robot at start-up. |
| `./run.sh --no-ros` | Does not start the ROS 2 bridge. |
| `./run.sh --headless --project <project.json>` | Runs without a window (for a server or CI). |
| `./run.sh --headless --no-cameras --duration 60 --urdf <robot.urdf>` | Runs 60 s of simulation time without an OpenGL context. |

### 2.1 Example projects

Each example directory contains a URDF and a `project.json` file.

| Directory in `robot_simulator/examples/` | Contents |
|---|---|
| `differential_drive_robot/` | Differential-drive base with velocity motors, `cmd_vel`, odometry, IMU, LiDAR, camera and an arena with walls |
| `robotic_arm/` | Arm with six joints, a fixed base, position motors and a camera on the wrist |
| `simple_manipulator/` | Revolute joint, prismatic joint and a gripper with a mimic joint, plus an IMU |
| `mesh_robot_description/` | ROS package with STL and OBJ meshes, `package://` and relative paths, and more than one visual for each link |

### 2.2 Mouse and keyboard

| Input | Action |
|---|---|
| Drag with the left mouse button | Turn the view around the target. |
| Drag with the right or middle mouse button | Move the view. |
| Mouse wheel | Zoom in or out. |
| Click | Select a robot. Click the robot again to select the link below the cursor. |
| `F` | Move the view to the selected item. |
| `Space` | Start or pause the simulation. |
| `S` | Do one physics step (only when paused). |
| `Ctrl+R` | Reset the simulation. |
| `G` / `C` | Show or hide the grid / the collision geometry. |
| `Ctrl+N` / `Ctrl+O` / `Ctrl+S` / `Ctrl+I` | New project / open project / save project / import URDF. |
| `Delete` | Remove the selected robot, object or sensor. |

### 2.3 Window layout

| Area | Contents |
|---|---|
| Toolbar (top) | Project commands, URDF import, environment objects, simulation controls, speed (0.25x to 4x). |
| Left panel | Scene hierarchy: environment, robots, links, joints and sensors. |
| Center | 3D viewport. |
| Right panel | Properties of the selected item. If no item is selected, the panel shows the physics and ROS 2 settings. |
| Bottom panel | Status line (simulation time, FPS, physics frequency, real-time factor, ROS 2 status), console, statistics and the list of ROS 2 topics. |
| Camera windows | Live images of the camera sensors. To open a window, select the camera sensor and click **Show image**. |

## 3. Architecture

### 3.1 Source code structure

| Package | Contents |
|---|---|
| `core/` | Application loops, simulation manager, clock, configuration, command queue, snapshots |
| `physics/` | PyBullet engine, robot connection, collision filters, motor commands |
| `robot/` | URDF parser, mesh and package resolver, robot hierarchy, loader |
| `motors/` | Rotational motor and linear motor |
| `control/` | Differential-drive controller (it uses rotational motors; it is not a motor type) |
| `sensors/` | Camera, IMU and 2D LiDAR |
| `rendering/` | Shaders, meshes, scene builder, ground and grid, sensor overlays, orbit camera |
| `ros2_bridge/` | Bridge thread, publishers, subscribers, services, TF |
| `environment/` | Environment objects |
| `project/` | Save and load of JSON projects |
| `ui/` | ImGui panels |
| `tests/` | pytest tests |

The dependencies go in one direction: `ui` → `core` → `physics`, `robot`, `sensors` and `environment`. The `rendering` package reads the simulation state. The `ros2_bridge` package uses only listener callbacks and the command queue of `core`. The `physics` package does not use the rendering, ROS 2 or the user interface. You can test each package alone.

### 3.2 Simulation loop and threads

```
main thread
  ├─ input (GLFW, ImGui)
  ├─ SimulationManager.update(wall_dt)
  │    ├─ execute the commands of the thread-safe command queue
  │    └─ accumulator: n = floor(acc / dt) fixed steps (dt = 1/240 s by default)
  │         for each step: motors → stepSimulation → sensors that are due → state snapshots and clock
  └─ OpenGL rendering (viewport and user interface)

ROS 2 bridge thread
  └─ rclpy executor: a guard condition starts the publication of the queued snapshots.
     The subscriber and service callbacks only add commands to the queue.
```

- **Fixed timestep:** The physics timestep does not change. The frame rate changes only the number of steps for each frame.
- **Slow computer:** If the computer is too slow, the simulator does not do more than `max_steps_per_frame` steps for each frame. It discards the remaining time. The simulation then runs slower than real time, but the window stays responsive. The status line shows the real-time factor.
- **Sensor rates:** Each sensor runs at its own rate in simulation time. A sensor does not update at each physics step.
- **ROS 2 does not block the physics:** The simulation thread only adds items to a queue with a size limit. If the bridge is too slow, the bridge discards the oldest items.

## 4. URDF support

| Item | Support |
|---|---|
| Joints | `fixed`, `revolute`, `continuous`, `prismatic`. The simulator keeps the parent, child, origin, axis, limits (lower, upper, velocity, effort) and dynamics (damping, friction). |
| Mimic joints | Supported. A position servo moves the joint to `multiplier × q_leader + offset`. The effort limit of the joint limits this servo. |
| `floating` and `planar` joints | Not supported. The simulator changes them to `fixed` and shows a warning. |
| Geometry | `box`, `cylinder`, `sphere` and `mesh` (STL, OBJ, DAE), with mesh `scale`. |
| Collision meshes | If PyBullet cannot read the mesh format, the simulator converts the mesh to OBJ in a cache. It does not change your files. On a dynamic link, PyBullet uses the convex hull of the mesh. |
| Mesh paths | Absolute paths, `file://`, paths relative to the URDF and `package://pkg/...`. |
| Materials | Global and inline colours. URDF `<texture>` tags are not supported. The simulator shows a warning and uses the colour. |
| Textured meshes | OBJ+MTL, DAE and glTF meshes with an image texture. The simulator uses the texture if the visual has no `<material>` colour. Use this method for QR codes and fiducial markers. |
| Links without `<inertial>` | The simulator gives them a mass of 1 g. PyBullet would otherwise give them 1 kg and unit inertia. |
| Fixed base | A robot with a root link named `world` gets a fixed base. For other robots, use **Import with fixed base** or the robot property. |

To find a `package://` path, the simulator searches these locations:

- `ROS_PACKAGE_PATH`
- `AMENT_PREFIX_PATH` (`share/<pkg>`)
- The directories above the URDF (folder name or `<name>` in `package.xml`)
- `package_paths` in `config.json`

The simulator shows a warning for each unsupported item. Examples are `transmission`, `gazebo`, `ros2_control`, `safety_controller`, `calibration`, unknown tags, unsupported geometry and missing meshes. The warnings are in the console and in the **Import warnings** list of the robot. The simulator does not ignore an item without a warning.

The robot keeps the `name` of its URDF. The simulator makes the name unique and valid for ROS. If two robots have the same TF frame names, the second robot gets the TF prefix `<name>/`.

## 5. Motors

The simulator does not assign motors automatically. To assign a motor, select a joint and click **Assign ... motor**. Fixed joints and mimic joints cannot have a motor.

| | Rotational motor | Linear motor |
|---|---|---|
| Joint types | revolute, continuous | prismatic |
| Position control | target position [rad] | target position [m] |
| Velocity control | target velocity [rad/s] | target velocity [m/s] |
| Effort control | torque [N·m] | force [N] |
| Limits | maximum torque, maximum velocity | maximum force, maximum velocity |
| Gains | position gain, velocity gain | position gain, velocity gain |

- Position control and velocity control use the joint motors of PyBullet. The gains are the normalized `positionGain` and `velocityGain` of PyBullet. The maximum effort limits the motor force.
- The URDF limits limit the target position.
- Effort control applies the limited torque or force at each step. At the maximum velocity, the motor stops to add effort in that direction.
- A joint without a motor is passive. The simulator applies the URDF `<dynamics friction>` as a friction torque or force.
- There are no electrical, hydraulic or pneumatic models.

### 5.1 Differential drive

To drive a mobile robot with `cmd_vel`, open the robot properties and go to **Differential drive**:

1. Select **Enable**.
2. Add the left wheel joints and the right wheel joints.
3. Set the wheel radius and the wheel separation.

The controller changes each `cmd_vel` command into wheel velocities: `ω = (v ∓ w·L/2) / r`. It sends these velocities to the rotational motors of the wheels. If no command comes in `cmd_vel_timeout_s` seconds, the wheels stop.

## 6. Sensors

You can attach a sensor to each link. Each sensor has an offset (xyz [m], rpy [rad]) and a rate [Hz].

By default, each sensor has its own frame `<name>_link`. A camera also has the frame `<name>_optical_frame`. The simulator publishes the static TF of these frames. If your robot already has a frame for the sensor, set `frame_id` to that frame. The simulator then uses this frame in the messages and publishes no TF for the sensor. For a camera, `frame_id` is the optical frame. Attach the camera to its body link (x forward).

### 6.1 Perfect IMU (`sensor_msgs/msg/Imu`)

- The orientation is the rotation of the IMU frame in the world frame.
- The angular velocity is the angular velocity of the link, in the IMU frame.
- The linear acceleration is the specific force `R^T (a − g)`. `a` is the difference of the velocity of the IMU point between two updates, divided by the time.
- A level IMU at rest gives `(0, 0, +9.81)`. In free fall, the IMU gives `0`.
- There is no noise, no bias and no drift. The covariances are zero.

### 6.2 2D LiDAR (`sensor_msgs/msg/LaserScan`)

- By default, the LiDAR does a 360° scan in the XY plane of the sensor. It uses batched `rayTestBatch` calls. `angle_min` is −π. The increment is 2π/N.
- If `horizontal_fov` is less than 2π, the scan is centered on +X. The beams go from −fov/2 to +fov/2, with the two limits. This is the same as `min_angle` and `max_angle` of a Gazebo `gpu_lidar`.
- A beam without a return gives `+inf` (REP-117).
- The LiDAR also sees the links of its own robot. Put the sensor where no link is in the scan plane.

### 6.3 RGB camera (`sensor_msgs/msg/Image`, `rgb8`, and `sensor_msgs/msg/CameraInfo`)

- The OpenGL renderer makes the images. The camera is an ideal pinhole camera: `fx = fy = (w/2) / tan(hfov/2)`, `cx = (w−1)/2`, `cy = (h−1)/2`, model `plumb_bob` with zero distortion.
- The body frame of the camera obeys REP-103 (x forward). The image header uses `<name>_optical_frame` (z forward, x right, y down).
- In headless mode, the camera uses a hidden OpenGL context. If no context is available, the simulator stops the cameras and shows a warning.

## 7. ROS 2 interface

The default node name is `/lightweight_sim`. You can change it in `config.json`. All time stamps use the simulation time. Start your nodes with `use_sim_time:=true`. In the tables, `<robot>` is the name of the robot.

### 7.1 Published topics

| Topic | Type | Rate | Notes |
|---|---|---|---|
| `/clock` | `rosgraph_msgs/msg/Clock` | 100 Hz (simulation time) | Set `publish_clock` to false to stop it. |
| `/tf` | `tf2_msgs/msg/TFMessage` | `state_publish_rate_hz` (50 Hz) | Movable joints, odom → root link |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | Latched (transient local) | Fixed joints, sensor mounts, optical frames |
| `/<robot>/joint_states` | `sensor_msgs/msg/JointState` | 50 Hz | All movable joints: position, velocity, effort |
| `/<robot>/odom` | `nav_msgs/msg/Odometry` | 50 Hz | Only if odometry is on for the robot |
| `/<robot>/imu/data` | `sensor_msgs/msg/Imu` | Sensor rate | |
| `/<robot>/scan` | `sensor_msgs/msg/LaserScan` | Sensor rate | |
| `/<robot>/camera/image_raw` | `sensor_msgs/msg/Image` | Sensor rate | `rgb8` |
| `/<robot>/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | With each image | |

If a robot has more than one sensor of the same type, the first sensor uses the topic in the table. The other sensors use `/<robot>/<sensor_name>/data`, `/<robot>/<sensor_name>/scan` or `/<robot>/<sensor_name>/image_raw`.

You can set a topic for each sensor. A relative name is in the robot namespace. A name that starts with `/` is absolute.

All publishers use reliable QoS. Thus, reliable subscribers and best-effort subscribers both receive the data.

### 7.2 Subscribed topics

| Topic | Type | Function |
|---|---|---|
| `/<robot>/cmd_vel` | `geometry_msgs/msg/Twist` (`TwistStamped` if `cmd_vel_stamped` is true) | `linear.x` [m/s] and `angular.z` [rad/s] go to the differential drive. Without a command for `cmd_vel_timeout_s`, the robot stops. |
| `/<robot>/joint_commands` | `sensor_msgs/msg/JointState` | Targets for the motors (refer to the rule below). |

Rule for `joint_commands`: For each `name[i]`, the simulator uses the first field that is not empty and not NaN. The sequence is `position[i]`, `velocity[i]`, `effort[i]`. The field sets the target and the control mode of the motor of that joint. The arrays can be empty. If a joint has no motor, the simulator ignores the command and shows a warning.

```bash
ros2 topic pub --once /six_dof_arm/joint_commands sensor_msgs/msg/JointState \
  "{name: [joint1, joint2], position: [0.5, -0.3]}"
ros2 topic pub /diff_bot/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.5}}"
```

### 7.3 Services and parameters

| Name | Type |
|---|---|
| `/sim/pause`, `/sim/resume`, `/sim/reset`, `/sim/stop`, `/sim/step` | `std_srvs/srv/Trigger` |
| `/sim/spawn_entity` | `simulation_interfaces/srv/SpawnEntity` (URDF `uri` or `resource_string`, `initial_pose`) |
| `/sim/delete_entity` | `simulation_interfaces/srv/DeleteEntity` |
| `/sim/set_simulation_state`, `/sim/get_simulation_state` | `simulation_interfaces/srv/SetSimulationState`, `GetSimulationState` |
| `/sim/reset_simulation` | `simulation_interfaces/srv/ResetSimulation` |
| `/sim/step_simulation` | `simulation_interfaces/srv/StepSimulation` (only when paused) |
| Parameter `sim_speed` (double) | Simulation speed, for example `ros2 param set /lightweight_sim sim_speed 2.0` |

The `simulation_interfaces` services are only available if that package is installed. The `std_srvs` services are always available.

### 7.4 Coordinate frames (REP-103, REP-105)

The frame names are the URDF link names. If the robot has a TF prefix, the frame names start with it.

```
[world] ─(static, optional: publish_world_tf)→ odom ─(/tf)→ <root link> ─ URDF tree ─┐
                                                       ├ fixed joints            (/tf_static, URDF origin)
                                                       ├ movable joints          (/tf, from the simulated link poses)
                                                       ├ <sensor>_link           (/tf_static, mount offset)
                                                       └ <camera>_optical_frame  (/tf_static)
```

- **Odometry on:** The tree is `odom → root link`. The `Odometry` message gives `base_frame` (any link) in the `odom` frame. The twist is in the base frame. The `odom` frame is at the initial pose of the robot. Thus, this odometry is ground truth.
- **Odometry off:** The simulator publishes `world → root link`. A mobile robot uses `/tf`. A robot with a fixed base uses `/tf_static`.
- **SLAM, AMCL and Nav2:** Keep `publish_world_tf` false. Then `map → odom` from SLAM or AMCL is the only parent of `odom`.
- **One parent for each frame:** Each frame has exactly one parent. The simulator does not accept a sensor name that gives the same frame as a frame of another robot.
- **robot_state_publisher:** By default, the simulator publishes the TF of the robot. Do not run `robot_state_publisher` for the same robot. If your project uses `robot_state_publisher`, set `ros.publish_tf` to false (refer to section 7.5).

### 7.5 ROS 2 settings for each robot

Use the `ros` and `odometry` items of the robot configuration in the project file.

| Setting | Values |
|---|---|
| `ros.namespace` | Not set: `/<robot name>`. `""`: root namespace. `"a/b"`: `/a/b`. |
| `ros.publish_tf` | `false`: the simulator publishes no TF for this robot. Use it if `robot_state_publisher` and a localizer publish the TF tree. |
| `ros.joint_states_topic`, `ros.cmd_vel_topic`, `ros.joint_commands_topic` | Topic names. A relative name is in the namespace. A name that starts with `/` is absolute. |
| `ros.enabled` | `false`: the robot has no ROS 2 interface. |
| `odometry.topic`, `odometry.child_frame` | Topic and `child_frame_id` of the odometry message. |
| `odometry.origin` | `spawn`: the odom frame is at the initial pose. `world`: the odometry is in the world frame (ground truth). |

## 8. Environment

To add an environment object, use the toolbar or the **Environment** menu. The object types are ground plane, box, cylinder, sphere, static wall and dynamic box. In the object properties, you can set:

- Position, rotation and dimensions
- Mass (0 = static object)
- Colour
- Collision on or off, visibility on or off
- Friction

## 9. Projects

A project is a directory:

```
project/
  project.json     name, physics, ROS 2, speed, environment and robots
                   (URDF path, pose, options, motors, sensors, drive, odometry,
                   initial joint positions) and view settings
  robots/  environments/  assets/  logs/
```

- **References:** The project refers to the URDF and mesh files. It does not copy them. Paths in the project directory are relative. Thus, you can move the directory. Other paths are absolute.
- **Missing files:** If a URDF or a mesh is missing, the simulator shows a clear error for that robot. It then loads the remaining items.
- **Safe save:** The simulator writes a temporary file and then renames it.

## 10. Configuration file (`robot_simulator/config.json`)

| Section | Settings |
|---|---|
| `physics` | `frequency_hz` (timestep = 1/`frequency_hz`), `gravity`, `solver_iterations`, `collision_detection`, `default_friction`, `default_restitution`, `contact_erp`, `max_steps_per_frame` |
| `ros2` | `enabled`, `node_name`, `state_publish_rate_hz`, `clock_publish_rate_hz`, `publish_clock`, `cmd_vel_timeout_s`, `cmd_vel_stamped`, `domain_id` |
| `render` | Window size, vsync, default overlays, background colour |
| `package_paths` | More directories to search for `package://` paths |

## 11. Tests

```bash
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
PYTHONPATH=$PWD:$PYTHONPATH python -m pytest robot_simulator/tests -q
```

The tests examine these items:

- **URDF:** parser, warnings and errors, mesh and package paths, forward kinematics.
- **Robots:** import of the example robots. The forward kinematics of the physics agree with the URDF forward kinematics.
- **Joints and motors:** revolute and prismatic motion, joint limits, velocity control, effort limits, motor rules, mimic joints, differential drive, `cmd_vel` time-out.
- **Sensors:** IMU orientation, angular velocity, gravity, centripetal acceleration, free fall. LiDAR ranges to known walls. Sensor rates. Camera image size, intrinsics and content.
- **Simulation loop:** the frame rate does not change the physics.
- **Projects:** save and load, relative paths, missing files.
- **ROS 2:** joint states, sensor topics, TF tree, `cmd_vel`, `joint_commands`, pause, resume, reset, spawn and delete.
- **Integration features:** namespaces, TF off, sensor frames, LiDAR field of view, world-frame odometry, textured meshes.

Without `rclpy`, the ROS tests are skipped. Without a display, the camera tests are skipped. The ROS tests use the ROS domain 77 and local discovery. Thus, they do not connect to other ROS 2 nodes on your computer. To use a different domain, set `ROBOT_SIM_TEST_DOMAIN_ID`.

## 12. Diagnostics

If the simulator stops responding, send the signal `SIGUSR1` to the process. The simulator then writes the stack of each thread to stderr:

```bash
kill -USR1 <pid>
```

## 13. Known limitations

- URDF `<texture>` tags are not rendered. Textured meshes are rendered (refer to section 4).
- On a dynamic link, a concave mesh collides as its convex hull. This is a PyBullet limitation.
- Self-collision and fixed base are load-time options. If you change them in the user interface, the simulator loads the robot again.
- PyBullet applies a small default velocity damping (0.04) to all bodies.
- PyBullet calculates the friction between two bodies as the product of the two friction coefficients. Some other simulators use the smaller coefficient.
- A reset sets the simulation time to 0. Nodes with `use_sim_time` then see a time that goes back. Other simulators do the same.
