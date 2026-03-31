# MAVLink Interface Reference

Distilled from AI Grand Prix Technical Specification (VADR-TS-001, Issue 00.01, 2026-03-09).

## Overview

The simulator communicates with your control software using MAVLink v2 over UDP via MAVSDK-compatible interfaces. You connect to a local UDP endpoint, exchange heartbeats, and then stream commands while receiving telemetry.

## Connection

- **Transport**: UDP
- **Protocol**: MAVLink v2
- **Client library**: MAVSDK (Python bindings: `mavsdk` package)

### MAVSDK Connection Pattern

```python
from mavsdk import System

drone = System()
await drone.connect(system_address="udp://:14540")  # typical SITL port

# Wait for connection
async for state in drone.core.connection_state():
    if state.is_connected:
        break
```

The exact port may vary. The sim documentation or launch script will specify it. Common SITL ports: 14540, 14550.

## Timing Constraints

| Parameter | Value |
|-----------|-------|
| Physics simulation rate | 120 Hz |
| Recommended command rate | 50–120 Hz |
| Minimum heartbeat rate | 2 Hz |
| Maximum run duration | 8 minutes (480 seconds) |

**Command rate matters.** At 50 Hz, you have 20ms between commands. At 120 Hz, you have 8.3ms. Higher command rates give smoother control but consume more CPU. Start at 50 Hz and increase if you see discretization artifacts in the flight path.

MAVSDK handles heartbeats automatically once connected. You do not need to manually send HEARTBEAT messages.

## Telemetry Messages (Simulator → Client)

### HEARTBEAT
Connection status. MAVSDK handles this transparently.

### ATTITUDE
Vehicle orientation and angular rates.

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| roll | float | rad | Roll angle (-π to π) |
| pitch | float | rad | Pitch angle (-π/2 to π/2) |
| yaw | float | rad | Yaw angle (-π to π) |
| rollspeed | float | rad/s | Roll rate |
| pitchspeed | float | rad/s | Pitch rate |
| yawspeed | float | rad/s | Yaw rate |

MAVSDK access:
```python
async for attitude in drone.telemetry.attitude_euler():
    roll_deg = attitude.roll_deg
    pitch_deg = attitude.pitch_deg
    yaw_deg = attitude.yaw_deg
```

### HIGHRES_IMU
Raw inertial measurement unit data.

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| xacc | float | m/s² | X acceleration |
| yacc | float | m/s² | Y acceleration |
| zacc | float | m/s² | Z acceleration |
| xgyro | float | rad/s | X angular rate |
| ygyro | float | rad/s | Y angular rate |
| zgyro | float | rad/s | Z angular rate |
| xmag | float | gauss | X magnetic field |
| ymag | float | gauss | Y magnetic field |
| zmag | float | gauss | Z magnetic field |

MAVSDK access:
```python
async for imu in drone.telemetry.imu():
    acc = imu.acceleration_frd  # forward-right-down
    gyro = imu.angular_velocity_frd
```

### ODOMETRY
Local position and velocity estimates in the NED frame. This is likely the most useful telemetry message for planning.

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| x | float | m | North position |
| y | float | m | East position |
| z | float | m | Down position (negative = up) |
| vx | float | m/s | North velocity |
| vy | float | m/s | East velocity |
| vz | float | m/s | Down velocity |
| q | float[4] | - | Orientation quaternion |

MAVSDK access:
```python
async for odom in drone.telemetry.odometry():
    pos = odom.position_body  # NED position relative to local origin
    vel = odom.velocity_body  # NED velocity (despite the "body" name)
```

**Field naming clarification**: Despite the field name `velocity_body`, in PX4 SITL this returns NED-frame velocities (north/east/down), not body-frame velocities (forward/right/down). These are the same only when the drone is yawed to heading 0° with no roll/pitch. During banked flight, NED velocity and body-frame velocity differ significantly. If you need body-frame velocity (e.g., for forward speed calculation), rotate the NED velocity by the inverse of the drone's attitude quaternion.

**Important**: The ODOMETRY message provides local position without GPS. This is your primary state estimate in the virtual qualifier. Verify its accuracy early — if drift is low, you can rely on it for planning. If drift is significant, you'll need to fuse it with vision data.

### TIMESYNC
Simulator time synchronization. Used to align your clock with the sim's clock. MAVSDK may handle this transparently, but be aware of it if you see timing inconsistencies.

### Telemetry Rate

The odometry stream updates at approximately 50 Hz in PX4 SITL. This bounds your effective control loop rate — `async for odom in drone.telemetry.odometry()` blocks until the next telemetry message arrives, so a 100 Hz command loop that reads odometry on every iteration will actually run at 50 Hz. If you need faster control, either: (a) run the command loop independently from the telemetry read (send commands at 100 Hz using the last known state), or (b) interpolate/predict the state between telemetry updates.

## Command Messages (Client → Simulator)

### SET_ATTITUDE_TARGET
Direct attitude control. You specify desired roll, pitch, yaw, and thrust.

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| q | float[4] | - | Desired attitude quaternion |
| body_roll_rate | float | rad/s | Desired roll rate (if using rate mode) |
| body_pitch_rate | float | rad/s | Desired pitch rate (if using rate mode) |
| body_yaw_rate | float | rad/s | Desired yaw rate (if using rate mode) |
| thrust | float | 0–1 | Collective thrust (0 = no thrust, 1 = full thrust) |
| type_mask | uint8 | - | Bitmask: which fields to ignore |

MAVSDK access (attitude mode):
```python
from mavsdk.offboard import Attitude

await drone.offboard.set_attitude(Attitude(
    roll_deg=10.0,      # bank right 10°
    pitch_deg=-5.0,     # nose down 5°
    yaw_deg=0.0,        # heading
    thrust_value=0.6    # 60% thrust (0.0–1.0 range)
))
```

**Hover thrust discovery**: The thrust_value for level hover is unknown a priori and varies by drone model. To find it: command level attitude (roll=0, pitch=0), start at thrust_value=0.3, and increment by 0.05 while observing vertical velocity from ODOMETRY. Hover thrust is the value where vertical velocity is approximately 0. For the PX4 SITL x500 model, hover thrust is typically around 0.5, but always verify empirically. Record the result in `notebook.md` — it's the foundation for all attitude-mode flight.

**When to use**: When you want direct control over the drone's orientation. Best for aggressive maneuvers where you need precise bank angles and thrust management. Requires you to manage altitude and speed yourself through the attitude commands. This is the Phase C control mode — highest performance ceiling but highest crash risk.

### SET_POSITION_TARGET_LOCAL_NED
Position and/or velocity setpoints in local NED frame.

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| x | float | m | North position target |
| y | float | m | East position target |
| z | float | m | Down position target (negative = up) |
| vx | float | m/s | North velocity target |
| vy | float | m/s | East velocity target |
| vz | float | m/s | Down velocity target |
| yaw | float | rad | Yaw target |
| type_mask | uint16 | - | Bitmask: which fields are active |

MAVSDK access (velocity mode):
```python
from mavsdk.offboard import VelocityNedYaw

await drone.offboard.set_velocity_ned(VelocityNedYaw(
    north_m_s=5.0,
    east_m_s=0.0,
    down_m_s=0.0,
    yaw_deg=90.0
))
```

**Warning about pure velocity mode**: `VelocityNedYaw` provides no position damping. The drone tracks the commanded velocity but has no concept of where it should be. This causes overshooting: the drone reaches the desired velocity but drifts past the target position. In practice, pure velocity mode is unstable for gate racing — use position+velocity feedforward instead.

MAVSDK access (position mode):
```python
from mavsdk.offboard import PositionNedYaw

await drone.offboard.set_position_ned(PositionNedYaw(
    north_m=10.0,
    east_m=5.0,
    down_m=-3.0,   # 3m above origin
    yaw_deg=45.0
))
```

MAVSDK access (position + velocity feedforward):
```python
from mavsdk.offboard import PositionNedYaw, VelocityNedYaw

# Combined position target with velocity hint
# Position provides alignment, velocity provides speed command
await drone.offboard.set_position_velocity_ned(
    PositionNedYaw(north_m=10.0, east_m=5.0, down_m=-3.0, yaw_deg=45.0),
    VelocityNedYaw(north_m_s=8.0, east_m_s=3.0, down_m_s=0.0, yaw_deg=45.0)
)
```

**Position+Velocity combined** is the key intermediate between pure position mode and attitude control. PX4 uses the velocity to inform how fast to fly toward the position target. The position target still provides the alignment and convergence behavior. Use this for Phase A+ speed management. Note: the exact behavior (how PX4 blends position and velocity) is empirical — test it.

**When to use**: When you want the sim's position/velocity controller to handle the low-level tracking. Simpler to use than attitude control. The sim's inner-loop controller will generate the attitude commands to track your setpoints. Good for initial development. May limit aggressiveness compared to direct attitude control because the inner-loop controller enforces its own safety margins.

## Control Mode Selection

You have two control modes. The choice between them is a key design decision:

**Position/Velocity mode** (SET_POSITION_TARGET_LOCAL_NED):
- Easier to implement. Send where you want to go, the sim figures out how.
- The sim's controller smooths out your commands — less risk of instability.
- May limit maximum performance because the sim's controller is tuned for stability, not racing.
- Good for Phase A and getting a baseline working.

**Attitude mode** (SET_ATTITUDE_TARGET):
- Full control over the drone's orientation and thrust.
- You manage the tradeoff between speed, altitude, and stability.
- Higher performance ceiling but higher crash risk.
- Requires you to understand the relationship between attitude, thrust, and trajectory.
- Good for Phase C when squeezing maximum performance.

**Hybrid approach**: Start with position/velocity mode in Phase A. In Phase C, consider switching to attitude mode for sections of the course where you need more aggressive flight, while keeping position mode for safer sections.

## Coordinate System

**NED (North-East-Down)**:
- X = North (positive forward in the initial heading)
- Y = East (positive right)
- Z = Down (positive toward ground, **negative = up**)

This is standard in aerospace. Be careful with the Z axis — to fly 3 meters high, set z = -3.0.

No GPS or global coordinates are available. All positions are relative to the sim's local origin (likely the start position).

## Offboard Mode

To send commands via MAVSDK, you must enter offboard mode:

```python
# Set an initial setpoint BEFORE starting offboard mode
await drone.offboard.set_position_ned(PositionNedYaw(0, 0, -2, 0))

# Start offboard control
await drone.offboard.start()

# Now your command loop runs...
```

If you stop sending commands, the flight controller may switch out of offboard mode (failsafe). Maintain your command rate.

## Vision Stream

A forward-facing FPV camera is available in the competition sim. Specifications:
- Approximately 12MP resolution
- Wide-angle lens
- Detailed parameters (resolution, FOV, encoding, frame rate, delivery mechanism) will be provided in a separate specification.

Gates are visually distinctive from the environment. Expect them to be a consistent color/shape throughout the Virtual Qualifier 1 track.

**PX4 SITL note**: The camera is NOT available in the default PX4 SITL x500 model used for development. The Gazebo sim can provide camera images, but this requires a drone model with a camera plugin (e.g., `gz_x500_depth` or a custom model). For Phase A and Phase A+, vision is not needed — odometry provides position. Phase B (perception integration) requires either: (a) setting up a camera in the Gazebo sim, or (b) waiting for the competition sim which includes the FPV camera. Plan accordingly.

## Practical Tips

1. **Start simple**: Get a connection working, read telemetry, hover in place. Then add waypoint following. Then optimize.
2. **Log everything**: In early experiments, log all telemetry to a file so you can analyze offline. This helps populate the lab notebook with empirical drone/sim characteristics.
3. **Async architecture**: MAVSDK is async (Python asyncio). Your main loop will be an async function that reads telemetry, computes commands, and sends them. Don't block the event loop with heavy computation — offload to a thread if needed.
4. **Command latency**: There will be some latency between sending a command and seeing its effect in telemetry. Measure this early — it affects how far ahead you need to plan.
5. **Telemetry bounds command rate**: Your effective control rate is bounded by the odometry update rate (~50 Hz), not your command send rate, if you read odometry on every loop iteration. See "Telemetry Rate" section above.
6. **Sim determinism**: The spec says environmental conditions are deterministic. In practice, expect ~0.5s run-to-run variance from async scheduling jitter. If you see variance > 1.0s, something is wrong.
