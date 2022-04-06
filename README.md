# robot_sim_diff_drive_rbs

**Version 0.1.3** (2022/04/06)

This project is a simulated differential drive robot intended to be used with
Gazebo. The robot is specified within a ROS package using URDF/Xacro files.

**With this version, it is possible to do:**

- Usage of the
  [`diff_drive_controller_rbs`](https://github.com/sousarbarb/diff_drive_controller_rbs)
  controller for differential drive robots
- 2D laser scanner on top of the robots
- Absolute paths when including files in a xacro file
- Launch file with an argument to select the intended URDF model
- Complete URDF model with odometry of a
  [two-wheeled differential drive robot](/urdf/diff_drive.urdf.xacro)
- Complete URDF model with odometry of a
  [four-wheel differential drive robot](/urdf/diff_drive_4.urdf.xacro)
  (aka skid-steer drive)

**The next version will add these features:**

- Fix namespace of the base laser scanner topic
- IMU

## Models

### [Two-wheeled differential drive robot](/urdf/diff_drive.urdf.xacro)

- Odometry working fine
- Two caster wheels simulated with two spheres (front and back to support the
  robot)

![Simulated Differential Drive Robot](/doc/img/diff_drive.png)

### [Four-wheeled differential drive robot](/urdf/diff_drive_4.urdf.xacro)

- Equivalent to a skid-steer drive robot
- Angular motion should make the robot slipping (bad for odometry)

![Simulated Skid Steer Drive Robot](/doc/img/diff_drive_4.png)

## ROS

**Current version:**

- Ubuntu 18.04 LTS 
- ROS Melodic

### Dependencies

- controller_manager
- diff_drive_controller_rbs
- gazebo_ros
- gazebo_ros_control
- joint_state_controller
- position_controllers
- robot_state_publisher
- rqt_robot_steering
- rviz
- xacro

### Parameters

- [`diff_drive.yaml`](/config/diff_drive.yaml) |
  [`diff_drive_4.yaml`](/config/diff_drive_4.yaml)
- [`joints.yaml`](/config/joints.yaml)

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
$ roslaunch robot_sim_diff_drive_rbs diff_drive.launch model:=<diff_drive|diff_drive_4>
```

## Contacts

- Ricardo B. Sousa ([gitlab](https://gitlab.com/sousarbarb/),
  [github](https://github.com/sousarbarb/),
  [personal](mailto:sousa.ricardob@outlook.com),
  [feup:professor](mailto:rbs@fe.up.pt),
  [feup:student](mailto:up201503004@edu.fe.up.pt),
  [inesctec](mailto:ricardo.b.sousa@inesctec.pt))
