# Porting guide: run your ROS 2 project on the simulator

This guide tells you how to run an existing ROS 2 robot project on the Lightweight Robotics Simulator. Most projects use Gazebo. The procedure is the same for other simulators.

You do not change the robot software (localization, Nav2, perception, behavior, mission). You replace only the parts that the old simulator supplied.

## Before you start

Make sure that you have these items:

- A URDF or xacro description of your robot.
- The list of topics that your nodes subscribe to and publish.
- The `controllers.yaml` file, if your project uses ros2_control.
- The world file of your project, if your robot needs a world.
- The simulator, installed as the [user guide](user_guide.md) tells you.

## Overview

| Item that Gazebo supplies | Usual source in a Gazebo project | Replacement in this simulator |
|---|---|---|
| `/clock` | `ros_gz_bridge` | The simulator publishes `/clock`. |
| Sensor topics (`/scan`, `/camera/*`, `/imu/data`) | `ros_gz_bridge` configuration | Simulator sensors with `topic` and `frame_id` (step 3) |
| `joint_states` | `joint_state_broadcaster` | The simulator publishes `joint_states`. |
| Wheel and joint control | `gz_ros2_control` and controllers | Simulator motors and the controller emulator (step 4) |
| Ground truth | Pose or odometry bridge | Simulator odometry with `origin: "world"` |
| World | SDF world file | Environment objects, or URDF files for objects with more than one shape (step 5) |
| Robot model | `robot_description` and `create` | The same URDF, loaded by the simulator |

## Step 1: Make a URDF file

The simulator reads only URDF. If your robot uses xacro, make a URDF file without the parts that are only for the old simulator:

```bash
xacro my_robot.urdf.xacro use_ros2_control:=false > my_robot.urdf
```

The argument names are different in each project. Look in your xacro file for the arguments that add `<ros2_control>` and `<gazebo>` blocks.

Notes:

- The simulator ignores `<gazebo>` and `<ros2_control>` tags and shows a warning for each tag. The tags do not cause a failure.
- The simulator finds `package://` meshes if the package is in a sourced workspace or in a directory above the URDF.
- Contact friction values from `<gazebo reference="...">` (`mu1`, `mu2`) go into `link_friction` of the robot configuration (step 3).

## Step 2: Write down the simulator interface

Make a table of the topics and frames that the old simulator supplied. To find them, use these commands while the old simulation runs:

```bash
ros2 topic list -t
ros2 topic info -v /scan
ros2 topic echo --once /scan --field header.frame_id
ros2 run tf2_tools view_frames
```

For each sensor, write down:

- the topic name and the message type
- the `frame_id`
- the rate
- the resolution and the field of view

Also write down which node publishes each TF edge.

## Step 3: Make the project file

A project file (`project.json`) contains the robots, their sensors and motors, the environment and the settings. You can write it by hand or with a script. A script is better, because your URDF, your sensor specifications and your world then stay the only data sources.

Example of one robot in the project file:

```jsonc
{
  "name": "my_robot",
  "urdf_path": "/abs/path/my_robot.urdf",
  "position": [0, 0, 0],
  "rpy": [0, 0, 0],
  "link_friction": {"left_wheel": 1.0, "caster": 0.01},
  "motors": [
    {"joint": "left_wheel_joint", "type": "rotational", "mode": "velocity",
     "max_effort": 20, "max_velocity": 10},
    {"joint": "right_wheel_joint", "type": "rotational", "mode": "velocity",
     "max_effort": 20, "max_velocity": 10}
  ],
  "sensors": [
    {"type": "lidar_2d", "name": "lidar", "link": "lidar_link", "frame_id": "lidar_link",
     "topic": "scan", "rate_hz": 10, "horizontal_fov": 3.316, "angular_resolution": 0.00873,
     "range_min": 0.1, "range_max": 12},
    {"type": "camera", "name": "camera", "link": "camera_link",
     "frame_id": "camera_optical_frame", "topic": "camera/image_raw",
     "width": 640, "height": 480, "horizontal_fov": 1.22, "rate_hz": 10},
    {"type": "imu", "name": "imu", "link": "imu_link", "frame_id": "imu_link",
     "topic": "imu/data", "rate_hz": 100}
  ],
  "odometry": {"enabled": true, "origin": "world", "topic": "ground_truth/odom",
               "odom_frame": "world", "child_frame": "ground_truth_base"},
  "ros": {"namespace": "", "publish_tf": false,
          "joint_states_topic": "joint_states",
          "cmd_vel_topic": "sim/cmd_vel",
          "joint_commands_topic": "sim/joint_commands"}
}
```

Put the robot in the `robots` list of a project document. For the full document format, look at `robot_simulator/examples/*/project.json`.

Set these items:

| Setting | Value |
|---|---|
| `ros.namespace` | `""` gives topics at the root (`/scan`, `/joint_states`). `"robot_1"` gives `/robot_1/...` for more than one robot. A topic name that starts with `/` does not use the namespace. |
| `ros.publish_tf` | Set `false` if `robot_state_publisher` and a localizer publish the TF tree. If the simulator also publishes TF, the tree has two parents for some frames. |
| `frame_id` of a sensor | Set the existing frame of your URDF (for example `lidar_link` or `camera_optical_frame`). The simulator then publishes no TF for the sensor. For a camera, attach the sensor to the body link (x forward) and set `frame_id` to the optical frame. |
| `topic` of a sensor | The topic that your nodes subscribe to. A relative name goes into `ros.namespace`. |
| `horizontal_fov` of a LiDAR | For a scan of less than 360°. The beams go from −fov/2 to +fov/2, with the two limits. Thus, `angular_resolution = fov / (samples − 1)`. |
| `odometry.origin` | `"world"` gives ground truth in the world frame, as a Gazebo pose bridge. Use it only for tests and evaluation. |
| `cmd_vel_topic`, `joint_commands_topic` | Put the command inputs of the simulator on topics that your project does not use. |

Make sure that the sensor rate, resolution, field of view and range agree with the values of step 2.

## Step 4: Replace ros2_control

The simulator has no controller manager. It accepts joint commands on `joint_commands_topic` (`sensor_msgs/JointState`) and it publishes `joint_states`. The `joint_states` messages contain position, velocity and effort. For a velocity motor, the effort is the force or torque of the actuator. Thus, load sensing from the effort works.

Use the controller emulator in `robot_simulator/examples/ros2_integration/`. It gives the same topics as the ros2_control controllers. It reads your `controllers.yaml` file.

| ros2_control controller | Emulator function |
|---|---|
| `joint_state_broadcaster` | Not necessary. The simulator publishes `joint_states`. |
| `diff_drive_controller` | Subscribes `<name>/cmd_vel`. Applies the velocity limits, the acceleration limits and the command time-out. Changes the command into wheel velocities. Publishes `<name>/odom` from the wheel encoder positions. It publishes no TF. |
| `joint_trajectory_controller` (velocity interface) | Subscribes `<name>/joint_trajectory`. Samples the trajectory. Sends `ff × v_ref + p × (q_ref − q)` as a velocity command. Holds the last point. |

Start the emulator:

```bash
python3 robot_simulator/examples/ros2_integration/controller_emulator.py --ros-args \
  -p use_sim_time:=true \
  -p controllers_file:=/path/to/controllers.yaml \
  -p base_controller:=diff_drive_base_controller \
  -p trajectory_controllers:="['arm_controller']" \
  -p joint_commands_topic:=/sim/joint_commands
```

Use the controller names of your `controllers.yaml` file. If your `cmd_vel` uses `geometry_msgs/Twist`, add `-p stamped_cmd_vel:=false`.

The emulator does not support all controller types. For another controller, write a node that uses the same topics and sends joint commands to the simulator. The file `controller_logic.py` shows the method. It has no ROS imports and you can test it without ROS.

## Step 5: Convert the world

The simulator has these environment objects: box, cylinder, sphere and plane.

1. For a static SDF world with boxes and planes, make one environment object for each collision box.
2. Calculate the pose of each object: model pose × link pose × collision pose.
3. For the colour, use the visual that has the same pose and size as the collision.
4. For a visual without a collision (for example a floor marking), make an object with `collision_enabled: false`.

For a movable object or an object with more than one shape (for example a pallet with fork pockets), write a URDF file without joints. Use one link with more than one collision box. Load this URDF as a robot. Set `"ros": {"enabled": false}`. If you need its ground truth, give it a namespace and odometry with `origin: "world"`.

### Textured surfaces (QR codes, fiducial markers)

To show an image on a surface, use a textured mesh:

1. Make an OBJ file with one square (two triangles) and texture coordinates.
2. Make an MTL file. Set `map_Kd` to the image file.
3. In the URDF visual, refer to the OBJ file. Do not add a `<material>` to this visual. The simulator then uses the texture.

Make sure that the image is not mirrored. When you look at the surface, the u axis of the texture must go to your right, and the v axis must go up. A mirrored QR code does not decode. Examine a camera image of the surface before you continue.

## Step 6: Add a backend selection to your launch files

Keep the old simulator as the default. Add a launch argument, for example `backend:=lightsim`.

For the world launch file:

1. Do not start the old simulator.
2. Make or select the project file.
3. Start the simulator:

   ```bash
   <simulator directory>/run.sh --project project.json --play            # with a window
   <simulator directory>/run.sh --project project.json --play --headless # without a window
   ```

For the robot launch file:

1. Do not start the spawn node (for example `ros_gz_sim create`) and the bridge node (`parameter_bridge`).
2. Do not start the controller spawners.
3. Start `robot_state_publisher` with the URDF of step 1.
4. Start the controller emulator.

Start all other nodes as before.

## Step 7: Keep your ROS graph separate

If other ROS 2 work runs on the same computer, use your own ROS domain:

```bash
export ROS_DOMAIN_ID=77                          # a domain that no other project uses
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST   # only for small graphs (refer to the note)
```

The simulator, your nodes and your terminals must use the same values.

Notes:

- `LOCALHOST` discovery uses a fixed list of local ports. In a test, a graph of approximately 50 processes was only partly discovered. For large graphs, keep your own `ROS_DOMAIN_ID` and do not set `LOCALHOST`.
- To stop your processes, use a pattern that is special to your project, for example the workspace path or the namespace. Do not stop processes by common names such as `ekf_node` or `robot_state_publisher`. Other projects also use these names.

## Step 8: Do the checks

Do these checks before you use the new setup:

- [ ] `ros2 topic list -t` shows the same sensor topics and types as the old simulator.
- [ ] The rates in simulation time are correct. Use a node that counts the header stamps. `ros2 topic hz` is too slow for large images.
- [ ] The `frame_id` of each sensor message is a frame in the TF tree.
- [ ] `ros2 run tf2_tools view_frames` shows one tree. No frame has two parents.
- [ ] Drive straight and turn in place. The wheel odometry, the localization and the ground truth agree.
- [ ] The actuator effort in `joint_states` agrees with the static load. For a lift, this is the carried mass × g.
- [ ] Camera-based perception works on the camera images of the simulator.

## Example result

The author used this procedure for a warehouse forklift project. The project uses Nav2, AMCL, an EKF, QR code perception, pallet handling by contact and a fleet of two robots. No node of the robot software changed.

| Mission | This simulator | Gazebo |
|---|---|---|
| One forklift, 3 boxes | Completed in 565 s | Completed in 578 s |
| Two forklifts, 3 boxes | Completed in 383 s, 0 conflicts | Completed in 382 s, 0 conflicts |
| Placement error (two forklifts) | 4.8 / 3.9 / 5.8 cm | 4.7 / 5.3 / 5.2 cm |
