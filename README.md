# robot_sim_diff_drive_rbs

**Version 0.0.4** (2022/03/30)

This project is a simulated differential drive robot intended to be used with
Gazebo. The robot is specified within a ROS package using URDF/Xacro files.

**With this version, it is possible to do:**

- Changed spawn of the robot to `z=0`
- Changed mass of wheels and body to more appropriate values
- Read automatically from URDF the odometry parameters
- Launch file to link ROS and Gazebo
- Gazebo launch file to spawn robot in an empty world
- Changed length of the model (0.6 to 0.3)
- Fixed bug in `<origin>` tag for `base_link`
- Fixed bug in the `wheel` xacro macro
- URDF model for a four-wheel differential drive robot
  ([diff_drive_4](/urdf/diff_drive_4.urdf.xacro))
- Fixed bug in package.xml (due to previously specifying format as 2)

**The next version will add these features:**

- Fix bug related to oscillating robot in simulation
- Fix bug related to the `base_link` being the center of the body for the 
  four-wheeled omnidirectional robot

## ROS

**Current version:**

- Ubuntu 18.04 LTS 
- ROS Melodic

### Dependencies

- controller_manager
- diff_drive_controller
- gazebo_ros
- gazebo_ros_control
- joint_state_controller
- position_controllers
- robot_state_publisher
- rqt_robot_steering
- rviz
- xacro

### Parameters

- TBD

### Publishes

- TBD

### Subscribes

- TBD

### Services

None.

### Actions

None.

## Usage

### Spawn robot in an empty world

```shell
$ roslaunch robot_sim_diff_drive_rbs gazebo.launch
```

### Simulate a differential drive robot

```shell
$ roslaunch robot_sim_diff_drive_rbs diff_drive.launch
```

## Contacts

- Ricardo B. Sousa ([gitlab](https://gitlab.com/sousarbarb/),
  [github](https://github.com/sousarbarb/),
  [personal](mailto:sousa.ricardob@outlook.com),
  [feup:professor](mailto:rbs@fe.up.pt),
  [feup:student](mailto:up201503004@edu.fe.up.pt),
  [inesctec](mailto:ricardo.b.sousa@inesctec.pt))
