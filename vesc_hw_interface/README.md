# Hardware Interface for VESC
This package provides us with hardware interfaces of VESC open source motor drivers running on ROS (Robot Operating System).
`vesc_hw_interface` libraries enable your robot to drive motors with VESC with [ros_control](http://wiki.ros.org/ros_control) framework.

## Features
- A simple controller drives a BLDC motor with VESC.
- Position (PID), velocity, current, and duty-cycle control are supported.
  - Calibration and joint limitation are implemented for position control.
  - You can give torque constant and gear ratio of your geared motor

### Limitations
* To prevent motors from breaking machines, the joint limitation should be implemented.
* Safety functions of this packages are NOT PERFECT. Please notice us if you find some mistakes in the implementation.

## Installation
Before build this package, you should install following packages additionally:
```bash
ros-$ROS_DISTRO-ros-control ros-$ROS_DISTRO-hardware-interface ros-$ROS_DISTRO-controller-manager
```

Then, clone this repository in `src` directory of your catkin workspace, and build with `catkin build` command. This package will be built with `vesc_driver` package, placed in the same repository.

## Usage
### Parameters
All of following parameters are in `${VESC_HW_INTERFACE_NODE_NAME}/` namespace.

#### Common
- `port` (string, **required**): port name connecting to your VESC, *e.g.* `/dev/ttyUSB0`.
- `command_mode` (string, **required**): control mode you want to use. Enter one of following parameters: `position`, `velocity`, `effort` and `effort_duty`.
- `joint_name` (string, *default*: `joint_vesc`): corresponding joint name in your robot URDF.
- `num_rotor_poles` (int, *default*: 2): the number of rotor poles.
- `gear_ratio` (double, *default*: 1.0): ratio of reduction calculated by `joint velocity/motor velocity`.
- `torque_const` (double, *default*: 1.0): motor torque constant (unit: Nm/A).
- `robot_description_name` (string, *default*: /robot_description): name of the robot description parameters for loading joint limits

**NOTE**: `gear_ratio` and `torque_const` are used to calculate joint states because VESC generally senses just motor position and motor current, neither joint position nor motor torque.
If your motor unit has other structures, you should implement your own controller.

#### For PID Position Control
- `servo/Kp` (double, *default*: 50.0): proportional gain of the controller.
- `servo/Ki` (double, *default*: 0.0): integral gain of the controller.
- `servo/Kd` (double, *default*: 1.0): derivative gain of the controller.
- `servo/calibration` (bool, *default*: true): if true, the servo will calibrate its origin position.
- `servo/calibration_mode` (string, *default*: "current"): the mode of calibration. Enter one of following parameters: `current`, `duty`. If you set `current`, the servo will calibrate its origin position with `calibration_current` and `calibration_strict_current`. If you set `duty`, the servo will calibrate its origin position with `calibration_duty` and `calibration_strict_duty`.
- `servo/calibration_current` (double, *default*: 6.0): maximum current used in origin calibration.
- `servo/calibration_strict_current` (double, *default*: same as `calibration_current`): maximum current used in strict origin calibration. If the servo reached the origin position with `calibration_current`, the servo moves slightly away from the origin position and then recalibrates the origin position with `calibration_strict_current`. If `calibration_strict_current` is same as `calibration_current`, the servo will not recalibrate the origin position.
- `servo/calibration_duty` (double, *default*: 0.1): maximum duty used in origin calibration.
- `servo/calibration_strict_duty` (double, *default*: same as `calibration_duty`): maximum duty used in strict origin calibration. If the servo reached the origin position with `calibration_duty`, the servo moves slightly away from the origin position and then recalibrates the origin position with `calibration_strict_duty`. If `calibration_strict_duty` is same as `calibration_duty`, the servo will not recalibrate the origin position.
- `servo/calibration_position` (double, *default*: 0.0): the position on which the robot calibrates.
- `servo/calibration_result_path` (string, *default*: ""): if not empty, the last position will be saved in this path.
- `servo/last_position` (double, *default*: 0.0): if `calibration` is false, the servo uses this value as its current position.
- `servo/use_endstop` (bool, *default*: false): if true, the servo will use the endstop sensor to check the position limit, and the servo subscribes `endstop` topic. You should publish `std_msgs/Bool` message to `endstop` topic. If the message is true, the servo will consider that the servo has reached the endstop.
- `servo/endstop_margin` (double, *default*: 0.02): the error margin for considering the target_position to have reached the endstop.
- `servo/endstop_window` (int, *default*: 1): the window size for considering the servo to have reached the endstop.
- `servo/endstop_threshold` (double, *default*: 0.8): the threshold for considering the servo to have reached the endstop. If the average of most recent `endstop_window` sensor values is greater than `endstop_threshold`, the servo will consider that the servo has reached the endstop.

**Warning**

- If you set `calibration` is true, the servo will move to the limit position with `calibration_current` current. Please make sure that the servo is safe to move.
- If you set `servo/calibration` is false, the servo will use `servo/last_position` as its current position. If you want to use the last position, you should save the last position in `servo/calibration_result_path` and set `servo/last_position` to the value.
- If you move the servo manually or use non hardware interface controller or didn't specify the `servo/calibration_result_path`, you should calibrate the servo again.

### Public Functions
- `void read()` sends request to get current states, but DOES NOT update immediately. After a return packet comes, the callback function will update private variables.
- `void write()` sends a command with specified mode.

### Samples

**position control**

```bash
roslaunch vesc_hw_interface position_control_sample.launch
```

**velocity control**

```bash
roslaunch vesc_hw_interface velocity_control_sample.launch
```

**effort control**

```bash
roslaunch vesc_hw_interface effort_control_sample.launch
```

## License
`vesc_hw_interface` is licensed under the [Apache 2.0 license](https://www.apache.org/licenses/LICENSE-2.0.html).
