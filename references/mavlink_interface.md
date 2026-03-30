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
    pos = odom.position_body  # NED position
    vel = odom.velocity_body  # NED velocity
```

**Important**: The ODOMETRY message provides local position without GPS. This is your primary state estimate in the virtual qualifier. Verify its accuracy early — if drift is low, you can rely on it for planning. If drift is significant, you'll need to fuse it with vision data.

### TIMESYNC
Simulator time synchronization. Used to align your clock with the sim's clock. MAVSDK may handle this transparently, but be aware of it if you see timing inconsistencies.

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
    thrust_value=0.6    # 60% thrust
))
```

**When to use**: When you want direct control over the drone's orientation. Best for aggressive maneuvers where you need precise bank angles and thrust management. Requires you to manage altitude and speed yourself through the attitude commands.

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

A forward-facing FPV camera is available. Specifications:
- Approximately 12MP resolution
- Wide-angle lens
- Detailed parameters (resolution, FOV, encoding, frame rate, delivery mechanism) will be provided in a separate specification.

Gates are visually distinctive from the environment. Expect them to be a consistent color/shape throughout the Virtual Qualifier 1 track.

The vision stream delivery mechanism is not yet specified. It may be a separate UDP stream, a shared memory interface, or accessible through MAVSDK. Prepare for any of these.

## Practical Tips

1. **Start simple**: Get a connection working, read telemetry, hover in place. Then add waypoint following. Then optimize.
2. **Log everything**: In early experiments, log all telemetry to a file so you can analyze offline. This helps populate the lab notebook with empirical drone/sim characteristics.
3. **Async architecture**: MAVSDK is async (Python asyncio). Your main loop will be an async function that reads telemetry, computes commands, and sends them. Don't block the event loop with heavy computation — offload to a thread if needed.
4. **Command latency**: There will be some latency between sending a command and seeing its effect in telemetry. Measure this early — it affects how far ahead you need to plan.
5. **Sim determinism**: The spec says environmental conditions are deterministic. This means if you run the exact same code twice, you should get very similar results. Small differences may come from floating-point non-determinism or timing jitter. If you see large run-to-run variance, something is wrong.
