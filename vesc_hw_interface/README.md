# vesc_hw_interface
A ROS2 ros2_control hardware interface for controlling BLDC motors using a [VESC (Vedder Electronic Speed Controller)](https://vesc-project.com/).
This interface supports multiple control modes, integrates with PID controllers, and is designed to work with common URDF joint types.

## Features
- ROS2 hardware interface for BLDC motors using VESC
- Supports the following command modes:
  - **Position control**
  - **Velocity control**
  - **Position duty control**
  - **Velocity duty control**
- PID controller integration (either using VESC internal PID or external PID within this interface)
- Homing support via current or duty mode

## Command Modes
This hardware interface supports multiple command modes. 
Each has its own behavior, PID control source, and supported URDF joint types.
| Command Mode | Description | PID Source | Supported Joint Types |
|--------------|-------------|------------|------------------------|
| `position` | Uses VESC's built-in position controller | VESC | `revolute`, `prismatic` |
| `velocity` | Uses VESC's built-in velocity controller | VESC | `continuous` |
| `position_duty` | Uses external PID controller in this interface to control duty cycle for position | External | `revolute`, `prismatic` |
| `velocity_duty` | Uses external PID controller in this interface to control duty cycle for velocity | External | `continuous` |

> [!NOTE]
> - **position**/**velocity** modes rely on **VESC internal PID**. Configure gains using VESC Tool.
> - **position_duty**/**velocity_duty** modes use **external PID control**, and PID gains must be set via URDF parameters.

> [!WARNING]
> URDF joint types must match the selected command mode; otherwise, the motor may behave unexpectedly.

> [!IMPORTANT]
> When using `position` mode, the joint range must map to a 0°–90° VESC range using the "Position Angle Division" setting. [More info.](#%EF%B8%8F-notes-on-vesc-pid-position-control)

## Parameters
Parameters should be set under the `<ros2_control>` tag in your URDF or `xacro` file.
Check the `vesc_hw_interface/launch` directory for examples.

### Common Parameters (All command mode)
| Name | Type | Requirement | Default | Description |
|------|------|-------------|---------|-------------|
| port | string | required | — | Serial port name to connect to the VESC |
| command_mode | string | required | — | Control mode: `"position"`, `"velocity"`, `"position_duty"`, `"velocity_duty"` |

### Parameters for `position` and `position_duty`
| Name | Type | Requirement | Default | Description |
|------|------|-------------|---------|-------------|
| servo/calibration | bool | optional | true | Runs homing operation before controller starts |
| servo/calibration_mode | string | optional | current | Homing mode: `"current"` or `"duty"` |
| servo/calibration_current | double | optional | 6.0 | Max current used in homing operation |
| servo/calibration_strict_current | double | optional | same as `calibration_current` | Max current used in strict homing operation. If the motor reaches the origin with `calibration_current`, it backs off and re-homes using this value. If equal to `calibration_current`, strict homing operation is skipped. |
| servo/calibration_duty | double | optional | 0.1 | Max duty used in homing operation |
| servo/calibration_strict_duty | double | optional | same as `calibration_duty` | Max duty used in strict homing operation. If the motor reaches the origin with `calibration_duty`, it backs off and re-homes using this value. If equal to `calibration_duty`, strict homing operation is skipped. |
| servo/calibration_position | double | optional | 0.0 | The position value assigned after homing operation |
| servo/calibration_result_path | string | optional | "" | If not empty, the last position is saved to this path |
| servo/last_position | double | optional | 0.0 | Used as the current position if `calibration` is false |
| servo/use_endstop | bool | optional | false | Enables endstop sensor support. Subscribes to the `endstop` topic (`std_msgs/Bool`). |
| servo/endstop_margin | double | optional | 0.02 | Position error margin to consider endstop as reached |
| servo/endstop_window | int | optional | 1 | Number of recent endstop readings used to evaluate endstop status |
| servo/endstop_threshold | double | optional | 0.8 | Threshold to determine endstop status based on average sensor values |

> [!NOTE]
> - If `calibration` is set to `true`, the servo will move to the limit using `calibration_current` or `calibration_duty`. Make sure it's safe for the servo to move.
> - If `servo/calibration` is set to `false`, the servo will use `servo/last_position` as its current position. To preserve the last position, save it using `servo/calibration_result_path` and reload it at startup.
> - If the servo is moved manually, controlled externally, or `servo/calibration_result_path` is unset, you should re-run calibration.

### Parameters for `position_duty` and `velocity_duty`
| Name | Type | Requirement | Default | Description |
|------|------|-------------|---------|-------------|
| num_hall_sensors | int | optional | 3 | Number of hall sensors | 
| num_rotor_poles | int | optional | 2 | Number of rotor poles |
| gear_ratio | double | optional | 1.0 | Gear reduction ratio |
| torque_const | double | optional | 1.0| Torque constant of the motor |
| screw_lead | double | optional | 1.0 | Screw lead (only used when joint type is `prismatic`) |

> [!NOTE]
> `position` and `velocity` command mode will use the gear ratio, num rotor poles, and torque constants from VESC.
Configure these parameters using VESC tool.

### Parameters exclusive to `position_duty`
| Name | Type | Requirement | Default | Description |
|------|------|-------------|---------|-------------|
| servo/Kp | double | optional | 50.0 | P gain |
| servo/Ki | double | optional | 0.0 | I gain |
| servo/Kd | double | optional | 1.0 | D gain |
| servo/i_clamp | double | optional | 1.0 | I clamp |
| servo/duty_limiter | double | optional | 1.0 | Duty cycle limiter |
| servo/antiwindup | bool | optional | true | Enable anti-windup |
| servo/control_rate | double | optional | 100.0 | PID control rate (Hz) |
| servo/enable_smooth_diff | bool | optional | true | Enable smoothing |
| servo/smooth_diff/max_sample_sec | double | optional | 1.0 | Max sampling window (sec) |
| servo/smooth_diff/max_smooth_step | int | optional | 10 | Max smoothing steps |

### Parameters exclusive to `velocity_duty`
| Name | Type | Requirement | Default | Description |
|------|------|-------------|---------|-------------|
| motor/Kp | double | optional | 0.005 | P gain |
| motor/Ki | double | optional | 0.005 | I gain |
| motor/Kd | double | optional | 0.0025 | D gain |
| motor/i_clamp | double | optional | 0.2 | I clamp |
| motor/duty_limiter | double | optional | 1.0 | Duty cycle limiter |
| motor/antiwindup | bool | optional | true | Enable anti-windup |
| motor/control_rate | double | optional | 100.0 | PID control rate (Hz) |
| motor/enable_smooth_diff | bool | optional | true | Enable smoothing |
| motor/smooth_diff/max_sample_sec | double | optional | 1.0 | Max sampling window (sec) |
| motor/smooth_diff/max_smooth_step | int | optional | 10 | Max smoothing steps |

## ⚠️ Notes on VESC PID Position Control
When using the `position` command mode, the hardware interface will use the VESC's internal PID position controller, which behaves as follows:
- VESC uses a circular (modular) angle representation, keeping position values within [0°, 360°). If the value exceeds 360° or drops below 0°, it wraps around accordingly.
- VESC moves the motor along the **shortest angular path**. For example, moving from 10° to 350° results in backward rotation.

This behavior is often undesirable in ROS, where we expect absolute angular movement.
To address this, the hardware interface remaps joint limits to align with the VESC’s internal behavior.

**This hardware interface assumes your motor’s joint limits correspond to a 0° to 90° range in VESC.**
You must set the **"Position Angle Division"** parameter in VESC Tool so that the physical joint motion between its lower and upper limits is scaled to exactly 90° in the VESC’s internal representation. (e.g., 0° is motor lower limit and 90° is motor upper limit)

This setup ensures the motor stays within a clean operating range and avoids unintended wrap-around or shortest-path movement.

## License
`vesc_hw_interface` is licensed under the [Apache 2.0 license](https://www.apache.org/licenses/LICENSE-2.0.html).
