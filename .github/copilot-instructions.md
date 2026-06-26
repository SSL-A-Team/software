# Software Repository — Copilot Context

## Overview

This is the ROS 2 (Jazzy) software stack for the A-Team SSL (Small Size League) robotics team. It orchestrates autonomous gameplay for a fleet of up to 16 omnidirectional-wheeled robots. The stack handles vision processing, game state tracking, AI strategy/play selection, motion planning, and radio communication with the physical robots (or a simulator).

The companion **firmware** repository runs on each robot's STM32H743 control board and implements a 1 kHz multi-mode position/velocity/acceleration motion controller with a 6-state Kalman filter, bang-bang trajectory planning, PID feedback, and closed-loop current (torque) control on 4 wheel motors.

**Build:** `colcon build` (from the `ateam_ws` workspace root, after sourcing `/opt/ros/jazzy/setup.bash`).
**Test:** `colcon test && colcon test-result --verbose`.
**Run (simulation):** `ros2 launch ateam_bringup bringup_simulation.launch.py`.
**Run (physical):** `ros2 launch ateam_bringup bringup_physical_game.launch.py`.

---

## Repository Structure

```
software/
├── ateam_bringup/          # Launch files for simulation and physical robot stacks
├── ateam_common/           # Shared C++ library (UDP helpers, topic names, GC listener, indexed topics)
├── ateam_geometry/         # Geometry primitives and operations (intersections, nearest points, arcs)
├── ateam_joystick_control/ # Manual joystick control node for individual robots
├── ateam_kenobi/           # AI strategy node (play selection, STP framework, motion planning)
├── ateam_msgs/             # Custom ROS message and service definitions
├── ateam_ui/               # Web-based UI (Neutralino + rosbridge) for visualization and control
├── motion/                 # Motion benchmarking & scripted scenarios
│   ├── ateam_motion_benchmark/         # C++ velocity/angular-velocity benchmarks
│   └── ateam_motion_scenarios/         # Python scripted motion-profiling scenarios
├── radio/                  # Radio bridge packages (real + simulation)
│   ├── ateam_radio_bridge/             # Physical robot radio bridge (WiFi/UDP)
│   ├── ateam_radio_msgs/               # Radio-specific ROS messages + software-communication submodule
│   └── ateam_ssl_simulation_radio_bridge/  # Simulator radio bridge (protobuf/UDP)
├── ssl_ros_bridge/         # Git submodule — bridges SSL vision and referee multicast to ROS topics
└── state_tracking/         # Vision filtering, field management, game state aggregation
    ├── ateam_field_manager/
    ├── ateam_game_state/
    └── ateam_vision_filter/
```

### Git Submodules
- `ssl_ros_bridge` — SSL league vision/referee to ROS bridge (`https://github.com/SSL-A-Team/ssl_ros_bridge.git`)
- `radio/ateam_radio_msgs/software-communication` — shared C packet definitions used by both this stack and firmware (`https://github.com/SSL-A-Team/software-communication.git`)

---

## System Architecture — Data Flow

```
SSL Vision Cameras (multicast 224.5.23.2:10006)
    │
    ▼
ssl_ros_bridge/vision_bridge ──► /vision_messages (VisionWrapper)
    │
    ▼
ateam_vision_filter ──► /ball (VisionStateBall)
                    ──► /blue_team/robot{0..15} (VisionStateRobot)
                    ──► /yellow_team/robot{0..15} (VisionStateRobot)
    │
    ▼
ateam_game_state ──► /world (GameStateWorld)
    │                    ▲
    │                    │
SSL Game Controller ─► ssl_ros_bridge/gc_bridge ──► /referee_messages (Referee)
    │
    ▼
ateam_kenobi (AI) ──► /robot_motion_commands/robot{0..15} (RobotMotionCommand)
    │
    ▼
ateam_radio_bridge ──► WiFi/UDP ──► Robot firmware (BasicControl packets)
    │                                       │
    │                                       ▼
    ◄── WiFi/UDP ◄── Robot firmware (BasicTelemetry / ExtendedTelemetry)
    │
    ▼
~/robot_feedback/basic/robot{0..15}      (BasicTelemetry)
~/robot_feedback/extended/robot{0..15}   (ExtendedTelemetry)
~/robot_feedback/connection/robot{0..15} (ConnectionStatus)
~/robot_feedback/error/robot{0..15}      (ErrorTelemetry)
```

---

## Radio Bridge — Physical Robots (`ateam_radio_bridge`)

### Discovery Protocol

Robots discover the software stack using **UDP multicast**:

1. **Robot sends `CC_HELLO_REQ`** to multicast group `224.4.20.69:42069` containing its `robot_id` and `TeamColor`, plus git hashes of firmware/controls/coms repos.
2. **Radio bridge receives** the discovery packet and replies with `CC_HELLO_RESP` containing the bridge's IP and port for that robot's dedicated connection.
3. **A per-robot `BiDirectionalUDP` connection** is established between the bridge and the robot's WiFi module (Odin W26x or Nora W36x).

Parameters:
- `discovery_address` (default `224.4.20.69`)
- `discovery_port` (default `42069`)
- `net_interface_address` (default `""` — bind to all interfaces)

### Radio Network Protocol (RNP)

All packets between the software stack and robots use the **RadioPacket** format defined in the `software-communication` submodule:

```c
RadioPacket (524 bytes max) = RadioHeader (8 bytes) + RadioData (516 bytes union)

RadioHeader {
    uint32_t crc32;          // CRC of everything after this field
    CommandCode command_code; // uint8_t enum
    uint8_t _reserved;
    uint16_t data_length;
}
```

#### Command Codes
| Code | Name | Direction | Payload |
|------|------|-----------|---------|
| 1 | `CC_ACK` | Bidirectional | — |
| 2 | `CC_NACK` | Bidirectional | — |
| 3 | `CC_GOODBYE` | Bidirectional | — |
| 4 | `CC_KEEPALIVE` | Bidirectional | — |
| 21 | `CC_HELLO_REQ` | Robot → Multicast | `HelloRequest` (robot_id, color, git hashes) |
| 22 | `CC_HELLO_RESP` | Software → Robot | `HelloResponse` (ipv4, port) |
| 41 | `CC_TELEMETRY` | Robot → Software | `BasicTelemetry` |
| 42 | `CC_CONTROL_DEBUG_TELEMETRY` | Robot → Software | `ExtendedTelemetry` |
| 43 | `CC_ROBOT_PARAMETER_COMMAND` | Bidirectional | `ParameterCommand` |
| 44 | `CC_ERROR_TELEMETRY` | Robot → Software | `ErrorTelemetry` |
| 61 | `CC_CONTROL` | Software → Robot | `BasicControl` |

### Command Sending

The radio bridge sends `CC_CONTROL` packets at a configurable rate (default **60 Hz**, parameter `command_frequency`).

For each connected robot, the bridge:
1. Takes the latest `RobotMotionCommand` from the `/robot_motion_commands/robot{id}` subscription
2. Fills a `BasicControl` struct with body control mode, skill command payload, kick/dribbler settings
3. Attaches the latest vision pose from `/blue_team/robot{id}` or `/yellow_team/robot{id}` (if fresher than `vision_state_staleness_ms`, default 100 ms)
4. Wraps in a `RadioPacket` with `CC_CONTROL` and sends via the per-robot UDP connection

If a robot's motion command topic hasn't been updated within `command_timeout_ms` (default 100 ms), a zero/off command is sent instead.

### Telemetry Reception

Incoming packets from robots are dispatched by command code:
- `CC_TELEMETRY` → published as `ateam_radio_msgs/BasicTelemetry` on `~/robot_feedback/basic/robot{id}`
- `CC_CONTROL_DEBUG_TELEMETRY` → published as `ateam_radio_msgs/ExtendedTelemetry` on `~/robot_feedback/extended/robot{id}`
- `CC_ERROR_TELEMETRY` → published as `ateam_radio_msgs/ErrorTelemetry` on `~/robot_feedback/error/robot{id}`
- `CC_ROBOT_PARAMETER_COMMAND` → forwarded to the firmware parameter server
- `CC_KEEPALIVE` → updates heartbeat timestamp

### Connection Management

Each robot has a `ConnectionState`: `Disconnected → Connecting → Connected → ReadyToClose`.

Parameters:
- `sustain_timeout_ms` (default 250) — how long without a packet before marking disconnected
- `connect_timeout_ms` (default 750) — timeout for initial connection handshake
- `connection_check_frequency` (default 10 Hz) — rate of connection state checks

Connection status is published on `~/robot_feedback/connection/robot{id}` as `ateam_radio_msgs/ConnectionStatus` (`bool radio_connected`).

### Controls Enable Flags

The radio bridge has parameters that control which firmware control loops are active:
- `controls_enabled.body_vel` (default `true`) — enable body velocity control
- `controls_enabled.wheel_vel` (default `true`) — enable wheel velocity feedforward
- `controls_enabled.wheel_torque` (default `true`) — enable wheel torque (current) control

These map to bit flags in the `BasicControl` packet: `wheel_vel_control_enabled` and `wheel_torque_control_enabled`.

---

## Radio Bridge — Simulation (`ateam_ssl_simulation_radio_bridge`)

For simulation, a separate bridge translates ROS commands into **SSL league protobuf** messages over UDP (no RNP protocol, no discovery):

- `ssl_sim_radio_ip` (default `127.0.0.1`)
- `ssl_sim_control_port` (default `10300`) — for simulator control commands (teleport ball/robots)
- `ssl_sim_blue_port` (default `10301`) — blue team robot commands
- `ssl_sim_yellow_port` (default `10302`) — yellow team robot commands

The simulation bridge:
- Subscribes to the same `/robot_motion_commands/robot{id}` topics
- Converts `RobotMotionCommand` → protobuf `RobotControl` with `RobotCommand` per robot
- Receives `RobotControlResponse` feedback and publishes as `BasicTelemetry`
- Publishes connection status (always connected when sim is running)
- Exposes `~/send_simulator_control_packet` service for teleporting objects

Note: Dribbler speed is scaled by `9.5492968` (rpm conversion), kick speed multiplied by `3.0` in sim conversion.

---

## Control Modes (Body Control Modes)

The `BodyControlMode` enum defines how the firmware interprets motion commands. This enum is shared between the software stack and firmware via the `software-communication` submodule.

| Mode | Value | Description | Fields Used |
|------|-------|-------------|-------------|
| `BCM_OFF` | 0 | Motors disabled, robot stops | None |
| `BCM_GLOBAL_POSITION` | 1 | Move to target pose in global frame | `pose.{x,y,theta}`, `limit_vel_{linear,angular}`, `limit_acc_{linear,angular}` |
| `BCM_GLOBAL_VELOCITY` | 2 | Track target velocity in global frame | `velocity.{x,y,theta}`, `limit_acc_{linear,angular}` |
| `BCM_LOCAL_VELOCITY` | 3 | Track target velocity in robot-local frame | `velocity.{x,y,theta}`, `limit_acc_{linear,angular}` |
| `BCM_GLOBAL_ACCEL` | 4 | Apply acceleration in global frame | `acceleration.{x,y,theta}` |
| `BCM_LOCAL_ACCEL` | 5 | Apply acceleration in robot-local frame | `acceleration.{x,y,theta}` |

### How Modes Are Used

- **ateam_kenobi** (AI node): Always publishes `BCM_LOCAL_VELOCITY`. The motion planning layer internally handles position intents, heading intents, and velocity intents, but the final output to the radio bridge is always a local velocity command.
- **ateam_joystick_control**: Always publishes `BCM_LOCAL_VELOCITY`.
- **Direct callers** can use any mode by publishing a `RobotMotionCommand` with the appropriate `body_control_mode` field.

### Firmware Behavior per Mode

- **`BCM_GLOBAL_POSITION`**: Requires active vision (200 ms timeout). Computes bang-bang trajectory to target pose, PID feedback using trajectory-vs-estimate twist error as derivative signal. Trajectory is recomputed on target change or tracking error exceeding thresholds.
- **`BCM_GLOBAL_VELOCITY`**: Computes bang-bang trajectory to target twist. PID on twist tracking error.
- **`BCM_LOCAL_VELOCITY`**: Rotates local target twist into global frame via z-rotation, then same as global velocity.
- **`BCM_GLOBAL_ACCEL` / `BCM_LOCAL_ACCEL`**: Direct passthrough — applies acceleration through dynamics model with no trajectory planning or PID.

All modes except `BCM_OFF` and the acceleration modes run through friction compensation (Coulomb + viscous) before the kinematic transform to wheel commands.

---

## Publishing Commands to Robots

### RobotMotionCommand Message (`ateam_msgs/msg/RobotMotionCommand`)

This is the primary interface for commanding robots. Publish to `/robot_motion_commands/robot{id}` (where `id` is 0–15):

```
uint8 body_control_mode     # BCM_OFF=0, BCM_GLOBAL_POSITION=1, BCM_GLOBAL_VELOCITY=2,
                             # BCM_LOCAL_VELOCITY=3, BCM_GLOBAL_ACCEL=4, BCM_LOCAL_ACCEL=5

Twist2D pose                # Target pose for BCM_GLOBAL_POSITION (x meters, y meters, theta radians)
Twist2D velocity            # Target velocity for velocity modes (m/s, rad/s)
Twist2D acceleration        # Target accel for accel modes (m/s², rad/s²)

float64 limit_vel_linear    # Max linear velocity (m/s), 0 = use firmware default (3.0)
float64 limit_vel_angular   # Max angular velocity (rad/s), 0 = use firmware default (3π)
float64 limit_acc_linear    # Max linear acceleration (m/s²), 0 = use firmware default (2.0)
float64 limit_acc_angular   # Max angular acceleration (rad/s²), 0 = use firmware default (2π)

uint8 kick_request          # KR_ARM=0, KR_DISABLE=1, KR_KICK_NOW=2, KR_KICK_TOUCH=3,
                             # KR_KICK_CAPTURED=4, KR_CHIP_NOW=5, KR_CHIP_TOUCH=6, KR_CHIP_CAPTURED=7
float32 kick_speed          # Kick/chip speed in m/s
float32 dribbler_speed      # Dribbler speed in RPM (default 300)
```

`Twist2D` is `{float64 x, float64 y, float64 theta}`.

### Example: Sending a Local Velocity Command

```bash
ros2 topic pub /robot_motion_commands/robot0 ateam_msgs/msg/RobotMotionCommand \
  "{body_control_mode: 3, velocity: {x: 0.5, y: 0.0, theta: 0.0}}"
```

### Example: Sending a Global Position Command

```bash
ros2 topic pub /robot_motion_commands/robot0 ateam_msgs/msg/RobotMotionCommand \
  "{body_control_mode: 1, pose: {x: 1.0, y: 0.5, theta: 1.57}}"
```

### Kick Requests

- `KR_ARM` (0): Arm the kicker (charge, ready to fire)
- `KR_DISABLE` (1): Disable kicker
- `KR_KICK_NOW` (2): Fire kick immediately
- `KR_KICK_TOUCH` (3): Fire kick when breakbeam detects ball
- `KR_KICK_CAPTURED` (4): Fire kick when ball is captured (breakbeam + dribbler)
- `KR_CHIP_NOW/TOUCH/CAPTURED` (5–7): Same but for chip kicks

### Safety Notes

- If no command is published for a robot within `command_timeout_ms` (100 ms), the radio bridge sends a zero/off command.
- The firmware has a 200 ms packet timeout — if no packet arrives, motors are stopped.
- `BCM_GLOBAL_POSITION` requires vision data; if vision is stale (>200 ms), the firmware outputs zero.

---

## Uploading Parameters to Robots

### Firmware Parameter Protocol

The radio bridge provides ROS services for reading and writing firmware parameters at runtime. This enables live tuning of control gains, Kalman filter noise, physical model parameters, and trajectory limits without reflashing.

### Services

- **`get_firmware_param`** (`ateam_msgs/srv/GetFirmwareParameter`)
  - Request: `int32 robot_id`, `int8 parameter_id`
  - Response: `float32[] data`, `bool success`, `string reason`

- **`set_firmware_param`** (`ateam_msgs/srv/SetFirmwareParameter`)
  - Request: `int32 robot_id`, `int8 parameter_id`, `float32[] data`
  - Response: `bool success`, `string reason`

### Parameter IDs

| ID | Name | Format | Fields |
|----|------|--------|--------|
| 0 | `KF_PROCESS_STD` | vec4 | `[pos_linear, pos_angular, vel_linear, vel_angular]` |
| 1 | `KF_MEASUREMENT_STD` | vec4 | `[vision_linear, vision_angular, encoder_angular, gyro_angular]` |
| 2 | `KF_MAX_STATE` | vec4 | `[max_pos_linear, max_pos_angular, max_vel_linear, max_vel_angular]` |
| 3 | `PHYS_WHEEL` | vec4 | `[alpha, beta, wheel_distance, wheel_radius]` |
| 4 | `PHYS_INERTIA` | vec2 | `[mass, moment_z]` |
| 5 | `PHYS_MOTOR_MODEL` | vec2 | `[torque_constant, efficiency_factor]` |
| 6 | `PHYS_FRICTION_MODEL` | vec4 | `[coulomb_linear, coulomb_angular, viscous_linear, viscous_angular]` |
| 7 | `COULOMB_COMP_ACCEL_DEADZONE` | f32 | `[deadzone_threshold]` |
| 8 | `POSE_CONTROL_GAIN` | vec2 | `[feedforward_gain, feedback_gain]` |
| 9 | `TRAJ_RECOMPUTE_ERROR` | vec4 | `[error_pos_linear, error_pos_angular, error_vel_linear, error_vel_angular]` |
| 10 | `POSE_FB_PIDII_LINEAR` | vec5 | `[P, I, D, I_min, I_max]` |
| 11 | `POSE_FB_PIDII_ANGULAR` | vec5 | `[P, I, D, I_min, I_max]` |
| 12 | `TWIST_FB_PIDII_LINEAR` | vec5 | `[P, I, D, I_min, I_max]` |
| 13 | `TWIST_FB_PIDII_ANGULAR` | vec5 | `[P, I, D, I_min, I_max]` |

### Protocol Details

1. The `FirmwareParameterServer` sends a `CC_ROBOT_PARAMETER_COMMAND` RadioPacket with a `ParameterCommand` payload.
2. `ParameterCommand` contains: `command_code` (`PCC_READ=0` or `PCC_WRITE=1`), `data_format` (scalar/vec2/vec3/vec4/vec5 of f32), `parameter_name`, and a `data` union.
3. The firmware processes the command and replies with `PCC_ACK` (echoing the written value back) or `PCC_NACK_INVALID_NAME` / `PCC_NACK_INVALID_TYPE_FOR_NAME`.
4. The bridge waits up to **300 ms** for a read reply and **100 ms** for a write reply.

### Example: Setting PID Gains

```bash
# Set pose PID linear gains [P=2.0, I=0.1, D=0.5, I_min=-1.0, I_max=1.0] on robot 3
ros2 service call /radio_bridge/set_firmware_param ateam_msgs/srv/SetFirmwareParameter \
  "{robot_id: 3, parameter_id: 10, data: [2.0, 0.1, 0.5, -1.0, 1.0]}"

# Read current twist PID angular gains from robot 0
ros2 service call /radio_bridge/get_firmware_param ateam_msgs/srv/GetFirmwareParameter \
  "{robot_id: 0, parameter_id: 13}"
```

---

## Power Management

### Service: `~/send_power_request`

Type: `ateam_radio_msgs/srv/SendRobotPowerRequest`
- Request: `int8 robot_id` (`-1` = all robots), `uint8 request_type` (`0` = reboot, `1` = shutdown)
- Response: `bool success`, `string reason`

Reboot/shutdown flags are embedded in the next `BasicControl` packet's bit fields (`request_shutdown`, `reboot_robot`).

---

## AI Strategy Node (`ateam_kenobi`)

### Architecture

Kenobi uses an **STP (Skills, Tactics, Plays)** framework:
- **Plays** are selected by a `PlaySelector` based on game state (referee commands, ball position, etc.)
- **Tactics** are assigned to individual robots within a play
- **Skills** produce `RobotCommand` structs with motion intents + kick/dribble commands

### Topics

| Direction | Topic | Type |
|-----------|-------|------|
| Subscribe | `/world` | `ateam_msgs/GameStateWorld` |
| Publish | `/robot_motion_commands/robot{0..15}` | `ateam_msgs/RobotMotionCommand` |
| Publish | `/overlays` | `ateam_msgs/OverlayArray` |
| Publish | `/play_info` | `ateam_msgs/PlayInfo` |
| Publish | `~/playbook_state` | `ateam_msgs/PlaybookState` |
| Publish | `~/status` | `ateam_msgs/KenobiStatus` |

### Services

| Service | Type |
|---------|------|
| `~/set_override_play` | `ateam_msgs/srv/SetOverridePlay` |
| `~/set_play_enabled` | `ateam_msgs/srv/SetPlayEnabled` |

### Motion Planning

Kenobi's internal motion planning uses intents:
- **Linear intents:** `VelocityIntent`, `PositionIntent`, `VelocityAtPositionIntent`
- **Angular intents:** `VelocityIntent`, `HeadingIntent`, `FacingIntent`, `FaceTravelIntent`

These are resolved by the `MotionExecutor` into a final `geometry_msgs/Twist`, which Kenobi wraps as a `BCM_LOCAL_VELOCITY` command.

### Playbook

- Loaded from a JSON file via the `playbook` parameter, or from an autosave cache.
- Individual plays can be enabled/disabled via the `set_play_enabled` service.
- A play override can be set via `set_override_play`.

---

## Joystick Control (`ateam_joystick_control`)

Manual control of individual robots via a gamepad.

### Topics

| Direction | Topic | Type |
|-----------|-------|------|
| Subscribe | `/joy` | `sensor_msgs/Joy` |
| Publish | `/robot_motion_commands/robot{id}` | `ateam_msgs/RobotMotionCommand` |
| Publish | `/joystick_control_status` | `ateam_msgs/JoystickControlStatus` |

### Parameters

- `robot_id` — which robot to control (`-1` disables, `0–15` selects robot)
- `command_topic_template` — topic name template with `{}` placeholder for robot ID
- `mapping.linear.x.axis/scale`, `mapping.linear.y.axis/scale`, `mapping.angular.z.axis/scale` — joystick axis mappings
- `mapping.kick`, `mapping.chip`, `mapping.dribbler.increment/decrement/spin` — button mappings
- `kick_speed`, `chip_speed`, `dribbler_speed_step`, `mapping.dribbler.max/min`

Always sends `BCM_LOCAL_VELOCITY` mode.

---

## Motion Scenarios (`ateam_motion_scenarios`)

Python (ament_python, no recompile) package under `motion/` for scripted motion-profiling routines. Each scenario is a single `rclpy` node driven by a state machine that publishes `RobotMotionCommand` messages directly to `/robot_motion_commands/robot{id}`. Tune behavior via ROS params at launch — no rebuild needed.

Shared, ROS-agnostic helper modules live in the `common/` subpackage (`ateam_motion_scenarios/common/`): `overlays.py` (`make_array`/`make_line`/`make_point`/`make_pose_marker`/`make_text` overlay builders, published to `/overlays` under a per-scenario `OVERLAY_NS`), `capture.py` (the shared two-phase `Capture` skill + `skill_capture_params.json` loader), and `pivot.py` (firmware `BCM_PIVOT` command builder + `PivotConfig` sourced from `skill_pivot_params.json`).

Each scenario loads its parameters from a `config/<scenario>_params.json` file (via a `_load_defaults()`/`_p()` helper, overridable with the `param_file` ROS param); `robot_id` and `team_color` live there too. When `require_connection` is true, scenarios start in a `WAIT_FOR_CONNECTION` state and restart their state machine if the robot's radio link drops mid-run.

### Scenarios

- **`ball_capture_scenario`** — Defaults to robot 2 / blue (configurable via `robot_id` / `team_color`). Subscribes `/ball`, `/blue_team/robot2`, `/field`. Drives the robot through:

  `WAIT_FOR_BALL → WAIT_FOR_STATIONARY → APPROACH_STAGING → CAPTURE → RETREAT_WITH_BALL → [STRAFE] → [AIM → KICK_AIMED | WAIT_BEFORE_KICK → KICK] → RESET → DONE (loops)`

  - **CAPTURE** target = `(ball_x - robot_front_offset + capture_overdrive, ball_y)` with `limit_vel_linear = capture_vel_limit` so the dribbler reaches the ball gently.
  - **STRAFE** issues `BCM_GLOBAL_VELOCITY` for `strafe_duration` seconds at `strafe_speed` along an angle measured from −y (positive = toward +x).
  - **AIM** continuously commands `BCM_GLOBAL_POSITION` at the robot's *current* xy with `theta = atan2(0 - ry, field_length/2 - rx)` and `limit_vel_angular = aim_max_angular_vel`. The "stay-at-current-xy" target acts as a brake while the robot rotates. Latches the pose and fires once yaw is within `yaw_tol` for `aim_dwell` seconds.
  - **RESET** drives to `(-field_length/2 + reset_margin, 0)` using `/field`.
  - Unified `pos_tol` and `yaw_tol` are used for every arrival check across all states.
  - Field positive-x is the +x goal; "behind" the ball means smaller x.
  - Run: `ros2 run ateam_motion_scenarios ball_capture_scenario` (override any param with `--ros-args -p name:=value`).

- **`pass_and_catch_scenario`** — Defaults to robot 2 / blue (configurable via `robot_id` / `team_color`). Captures a stationary ball, aims at a configurable point on a field-edge "wall", kicks, then drives to intercept the geometrically reflected ricochet (perfectly elastic bounce, no friction/spin). State machine:

  `WAIT_FOR_BALL → WAIT_FOR_STATIONARY → ALIGN_STAGING_Y → APPROACH_STAGING → CAPTURE → AIM_AT_WALL → KICK → DRIVE_TO_INTERCEPT → CATCH_DWELL → PIVOT_BACK → KICK_BACK → RESET → DONE (loops)`

  - Wall is one of `pos_y` / `neg_y` (touchlines, reflect y) or `pos_x` / `neg_x` (goallines, reflect x). Hit point parameterized by `wall_hit_offset` (signed, along the wall from field center); `wall_inset` shifts inward to account for ball radius.
  - Intercept pose places the dribbler `intercept_distance` meters along the reflected ray from the bounce point, facing the bounce so the ball runs into the dribbler.
  - After catching, **PIVOT_BACK** rotates in place to `pivot_heading` (default 0 = +x goal) and **KICK_BACK** kicks the ball back.
  - Run: `ros2 run ateam_motion_scenarios pass_and_catch_scenario`.

### Conventions for adding new scenarios

- New scenarios go in `motion/ateam_motion_scenarios/ateam_motion_scenarios/` as Python modules, added as new entry points in `setup.py`. Put shared, reusable, ROS-agnostic logic in the `common/` subpackage instead of duplicating it per scenario.
- Load parameters from a per-scenario `config/<scenario>_params.json` via the `_load_defaults()`/`_p()` helper; keep `robot_id` / `team_color` (and other tuning) in that JSON rather than hardcoding module-level constants.
- Support `require_connection`: start in `WAIT_FOR_CONNECTION` and reset the state machine if the radio link drops (`ConnectionStatus` on `~/robot_feedback/connection/robot{id}`).
- Always send `BCM_GLOBAL_POSITION`, `BCM_GLOBAL_VELOCITY`, or the firmware `BCM_PIVOT` from these scenarios; reserve `BCM_LOCAL_VELOCITY` for kenobi/joystick.
- Reuse the `pos_tol` / `yaw_tol` pattern instead of introducing per-phase tolerances.
- Leave limit fields at `0.0` when you want the firmware defaults; only set positive values to clamp.
- Use the shared `common/overlays.py` helpers to publish debug visualizations to `/overlays` rather than rolling new overlay builders.
- Run kenobi off (or disable the target robot in the playbook) before running a scenario, since both will publish to the same `/robot_motion_commands/robot{id}` topic.

---

## State Tracking Pipeline

### Vision Filter (`ateam_vision_filter`)
- Subscribes: `/vision_messages` (raw SSL vision), `/field`
- Publishes: `/ball` (VisionStateBall), per-robot vision on `/blue_team/robot{id}` and `/yellow_team/robot{id}` (VisionStateRobot), `/vision_state` (debug)
- Uses multi-hypothesis tracking (MHT) with interacting multiple model (IMM) filters

### Field Manager (`ateam_field_manager`)
- Subscribes: `/vision_messages`
- Publishes: `/field` (FieldInfo)
- Service: `~/set_ignore_field_side`
- Persists field side preference to disk

### Game State Tracker (`ateam_game_state`)
- Subscribes: `/field`, `/ball`, robot vision topics, robot feedback/connection topics, robot motion commands, referee messages
- Publishes: `/world` (GameStateWorld) — the unified world state consumed by ateam_kenobi
- Aggregates all sensor data, referee info, and robot status into a single world model

---

## Standard Topic Names

Defined in `ateam_common/topic_names.hpp`:

| Topic Pattern | Type | Description |
|---------------|------|-------------|
| `/vision_messages` | `ssl_league_msgs/VisionWrapper` | Raw SSL vision data |
| `/referee_messages` | `ssl_league_msgs/Referee` | Game controller commands |
| `/ball` | `ateam_msgs/VisionStateBall` | Filtered ball state |
| `/blue_team/robot{0..15}` | `ateam_msgs/VisionStateRobot` | Blue team robot vision |
| `/yellow_team/robot{0..15}` | `ateam_msgs/VisionStateRobot` | Yellow team robot vision |
| `/field` | `ateam_msgs/FieldInfo` | Field geometry |
| `/world` | `ateam_msgs/GameStateWorld` | Unified world state |
| `/robot_motion_commands/robot{0..15}` | `ateam_msgs/RobotMotionCommand` | Robot commands |
| `/robot_feedback/basic/robot{0..15}` | `ateam_radio_msgs/BasicTelemetry` | Basic telemetry |
| `/robot_feedback/extended/robot{0..15}` | `ateam_radio_msgs/ExtendedTelemetry` | Extended telemetry |
| `/robot_feedback/connection/robot{0..15}` | `ateam_radio_msgs/ConnectionStatus` | Connection status |
| `/joy` | `sensor_msgs/Joy` | Joystick input |
| `/joystick_control_status` | `ateam_msgs/JoystickControlStatus` | Joystick active status |
| `/overlays` | `ateam_msgs/OverlayArray` | Debug visualization overlays |

---

## Launch Configurations

### `bringup_simulation.launch.py`
Starts the full simulation stack:
- ER-Force simulator (`simulator-cli`)
- SSL game controller (Docker)
- SSL vision/referee ROS bridges
- Vision filter + field manager + game state
- Simulation radio bridge
- ateam_kenobi (AI)
- ateam_ui (web interface)
- Joystick controller

Arguments: `start_sim`, `start_gc`, `start_ui`, `team_name`

### `bringup_physical_core.launch.py`
Starts infrastructure for physical robots (no AI):
- SSL vision/referee ROS bridges
- Vision filter + field manager + game state
- Physical radio bridge
- ateam_ui
- Joystick controller
- WiFi checker

### `bringup_physical_game.launch.py`
Wraps `bringup_physical_core` and adds ateam_kenobi for autonomous play.

### `joystick_only_stack.launch.py`
Minimal stack for joystick-only manual control:
- Joystick controller + physical radio bridge

---

## UI (`ateam_ui`)

Web-based interface built with Neutralino, communicating via rosbridge WebSocket.

### Key UI Features
- Field visualization (robots, ball, overlays)
- Goalie picker → `set_desired_keeper` service
- Playbook controls → `set_play_enabled`, `set_override_play` services
- Field side toggle → `set_ignore_field_side` service
- Power controls → `send_power_request` service (reboot/shutdown robots)
- Ball/robot teleportation in simulation → `send_simulator_control_packet` service
- Joystick robot selection → writes `robot_id` parameter on joystick node
- Robot status display (battery, connection, errors)

---

## Key Message Types (Quick Reference)

### `GameStateWorld`
```
current_time, field (FieldInfo), referee_info (RefereeInfo), ball (GameStateBall),
our_robots[] (GameStateRobot), their_robots[] (GameStateRobot),
ball_in_play, double_touch_enforced, double_touch_id
```

### `GameStateRobot`
```
id, visible, radio_connected, pose, velocity, prev_command_velocity,
breakbeam_ball_detected, breakbeam_ball_detected_filtered, kicker_available, chipper_available
```

### `BasicControl` (C struct sent over radio)
```
Bit flags: request_shutdown, reboot_robot, game_state_in_stop, emergency_stop,
           wheel_vel_control_enabled, wheel_torque_control_enabled, vision_update, reset_controller
Vision: vision_position_update[3] (x, y, θ)
Mode: body_control_mode (BodyControlMode enum)
Kicker: kick_request (KickRequest enum), kick_vel (m/s), dribbler_speed (RPM)
Command: cmd (BodyControlCommand union — varies by mode)
Total size: 56 bytes
```

### `BasicTelemetry` (C struct from robot)
```
Sequence numbers, body_control_mode
Status bit flags: power/battery/motor/kicker/sensor errors, breakbeam, kicker availability
battery_percent, kicker_charge_percent
Per-mode telemetry union
Total size: 16 bytes
```

### `ExtendedTelemetry` (C struct from robot)
```
Timestamp, PowerTelemetry (battery cell voltages/percentages)
Per-motor CcmTelemetry (4× — encoder velocity, current, bus voltage, temperature)
BodyControlExtendedTelemetry (KF states, trajectory setpoints, control outputs, IMU, vision)
KickerTelemetry (charge, ball detected, dribbler motor)
Total size: 516 bytes
```

---

## Game Controller Integration

The `GameControllerListener` (in `ateam_common`) subscribes to `/referee_messages` and provides:
- Team color (blue/yellow) — determines which vision topics and simulator ports to use
- Team side (positive/negative x)
- Current/previous/next referee command
- Goalie IDs
- Designated position (for ball placement)

Parameters:
- `gc_team_name` — team name to match in referee messages
- `default_team_color` — fallback if team not found in referee
- `default_team_side` — fallback side assignment
