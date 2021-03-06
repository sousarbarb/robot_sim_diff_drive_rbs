# History

## Version 0

### Version 0.1

_(summary)_

- Complete URDF model with odometry of a
  [two-wheeled differential drive robot](/urdf/diff_drive.urdf.xacro)

**Version 0.1.4** (2022/04/07)

- Fixed gravity bug of having only 1 simulated caster wheel for `diff_drive`
- `diff_drive` model compatible with different radius for each wheel (left
  and right)

**Version 0.1.3** (2022/04/06)

- Usage of the
  [`diff_drive_controller_rbs`](https://github.com/sousarbarb/diff_drive_controller_rbs)
  controller for differential drive robots

**Version 0.1.2** (2022/04/01)

- 2D laser scanner on top of the robots
- Absolute paths when including files in a xacro file

**Version 0.1.1** (2022/04/01)

- Launch file with an argument to select the intended URDF model

**Version 0.1.0** (2022/04/01)

- Complete URDF model with odometry of a
  [two-wheeled differential drive robot](/urdf/diff_drive.urdf.xacro)

### Version 0.0

_(summary)_

- Complete URDF model with odometry of a
  [four-wheel differential drive robot](/urdf/diff_drive_4.urdf.xacro)
  (aka skid-steer drive)

**Version 0.0.5** (2022/04/01)

- Parametric inertia models
- Fix bug related to oscillating robot in simulation
- Fix bug related to the `base_link` being the center of the body for the
  four-wheeled omnidirectional robot

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
