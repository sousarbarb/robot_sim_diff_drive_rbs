# History

## Version 0

### Version 0.0

**Version 0.0.4** (2022/03/30)

- Changed spawn of the robot to `z=0`
- Changed mass of wheels and body to more appropriate values
- Read automatically from URDF the odometry parameters
- Launch file to link ROS and Gazebo

**Version 0.0.3** (2022/03/30)

- Gazebo launch file to spawn robot in an empty world
- Changed length of the model (0.6 to 0.3)
- Fixed bug in `<origin>` tag for `base_link`
- Fixed bug in the `wheel` xacro macro

**Version 0.0.2** (2022/03/30)

- URDF model for a four-wheel differential drive robot
  ([diff_drive_4](/urdf/diff_drive_4.urdf.xacro))

**Version 0.0.1** (2022/03/30)

- Fixed bug in package.xml (due to previously specifying format as 2)

**Version 0.0.0** (2022/03/30)

- Initialization of the Git repository
- Initialization of the ROS package
