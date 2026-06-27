# Jetvio Drone — GPS-denied warehouse navigation

A ROS 2 autonomy stack for a quadrotor that navigates warehouse aisles without GPS.
Visual-inertial odometry (cuVSLAM) provides the pose estimate, EGO-Planner produces
obstacle-aware trajectories from depth, and a guarded bridge converts those
trajectories into PX4 setpoints. A three-piece failsafe detects VIO dropout and
hands the vehicle to an optical-flow-held hover under a safety pilot.

## Hardware / platform

- **Compute:** NVIDIA Jetson Orin NX 16 GB (JetPack 6.2.1, Ubuntu 22.04)
- **Camera:** Intel RealSense D455 (IR stereo + depth + IMU)
- **Autopilot:** Pixhawk 6X Pro, PX4, connected over TELEM2 → `/dev/ttyTHS1` @ 921600
- **Optical-flow fallback:** Holybro H-Flow (PixArt flow + Broadcom ToF + IMU) over DroneCAN
- **Stack:** ROS 2 Humble, Isaac ROS 3.2 (cuVSLAM / nvblox / RealSense), CUDA 12.6

## System overview

```
RealSense D455
      |
      v
  Perception  ── realsense_splitter ── cuVSLAM (VIO odom, 15 Hz)
      |                                     |
      |                                     +── depth (848x480) ──> EGO-Planner
      |
  vio_health_monitor ── /vio/health
      |                    |
      |          (gates)   +──────────────┐
      v                                   v
  vio_mavros_relay ──> /mavros/vision_pose/pose ──> PX4 EKF2
                                                       ^
  EGO-Planner ── /ego/pos_cmd ──> setpoint_bridge ────┘
                                  (/mavros/setpoint_raw/local)
```

Two data paths leave perception: the **VIO path** (cuVSLAM → external-vision relay →
EKF2) supplies PX4's pose estimate, and the **depth path** feeds EGO-Planner's local
occupancy map. EGO's trajectory is consumed only by `setpoint_bridge`, the single node
permitted to command the vehicle.

## Packages

| Package | Build | Role |
|---|---|---|
| `jetvio_perception` | ament_cmake | Camera + splitter + cuVSLAM launch; emitter-heal helper |
| `jetvio_localization` | ament_cmake | cuVSLAM map load / relocalization (scaffold — not yet implemented) |
| `jetvio_mapping` | ament_cmake | nvblox occupancy load/replay launches |
| `jetvio_planning` | ament_cmake | EGO-Planner live launch + camera-pose relay |
| `jetvio_flight` | ament_python | VIO relay, setpoint bridge, health monitor, EKF origin setter |
| `jetvio_description` | ament_cmake | Static camera ↔ base_link transform |
| `jetvio_bringup` | ament_cmake | Top-level orchestration launch + Foxglove launch |
| `realsense_splitter` | ament_cmake | Vendored nvblox splitter (infra + depth fan-out) |

The stack spans two workspaces: this one (`~/jetvio_ws`) and `~/ego_ws`
(ego-planner-swarm, GPLv3). `quadrotor_msgs` lives in `ego_ws`, so any node that
touches `/ego/pos_cmd` needs both workspaces sourced.

## Build

```bash
cd ~/jetvio_ws
colcon build --symlink-install
source install/setup.bash
```

`--symlink-install` means Python node edits are live without rebuilding; rebuild only
after adding entry points or changing C++ / manifests.

## Running

Source prefix (add `~/ego_ws` for anything using `quadrotor_msgs`):

```bash
source /opt/ros/humble/setup.bash && source ~/jetvio_ws/install/setup.bash
# + && source ~/ego_ws/install/setup.bash   # for EGO and setpoint_bridge
```

### 1. Bring up the flight stack

```bash
ros2 launch jetvio_bringup bringup.launch.py reboot_fc:=false
```

Starts, in order: camera + splitter → emitter heal → **(on heal exit)** cuVSLAM →
MAVROS → EKF origin → VIO health monitor → gated VIO relay. cuVSLAM is load-ordered
after the emitter heal so the IR-emitter toggle never disrupts initial convergence.

Launch args:
- `fcu_url` (default `/dev/ttyTHS1:921600`) — PX4 serial link
- `reboot_fc` (default `true`) — reboot the autopilot for a fresh EKF2 before setting origin

cuVSLAM needs a **textured scene with some parallax** to converge — point the camera at
shelving / structure, not a blank wall. A static camera on a featureless surface is a
hard, non-self-recovering VIO failure mode (this is exactly what the flow fallback
exists for).

### 2. Planning (both workspaces sourced)

```bash
ros2 launch jetvio_planning ego_live.launch.py
```

Send a goal in clear space (frame `odom`):

```bash
ros2 topic pub --once /move_base_simple/goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'odom'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"
```

### 3. Setpoint bridge (both workspaces sourced)

Bench (gate bypassed, ground command in-box):

```bash
ros2 run jetvio_flight setpoint_bridge --ros-args -p bypass_gate:=true -p z_min:=-1.0
```

Enable following:

```bash
ros2 service call /setpoint_bridge/follow_enable std_srvs/srv/SetBool "{data: true}"
```

### 4. Visualization (optional, separate terminal)

```bash
ros2 launch jetvio_bringup foxglove.launch.py
```

Connect Foxglove Studio to `ws://<jetson-ip>:8765`, fixed frame `odom`. Kept separate
from `bringup` on purpose — the flight graph stays minimal and deterministic.

## Safety architecture

### Setpoint bridge guard layers (precedence per tick)

`setpoint_bridge` is the only node that commands the vehicle. It never arms or changes
mode. Each control cycle, in order:

1. **EGO-loss failsafe** — `/ego/pos_cmd` absent > `fail_timeout` (2.0 s) → stop
   publishing (latched); PX4's offboard-loss action takes over.
2. **VIO-health gate** — `/vio/health` degraded or stale → freeze to a held position,
   disable follow. Recovery does **not** auto-resume; re-enable is required.
3. **Hover-hold** — not enabled / not armed+offboard → stream current pose.
4. **Follow** — transformed EGO setpoint (pos + vel + yaw feedforward), bounds- and
   speed-guarded, hold-last on brief staleness.

### Three-piece VIO-dropout failsafe

cuVSLAM's `vo_state` flag is unreliable (it can report success while tracking is lost),
so detection reads the odometry and covariance streams directly:

1. **`vio_health_monitor`** — observes only; publishes `std_msgs/Bool` on `/vio/health`
   at 10 Hz. Signals: orientation covariance (leading indicator), position/yaw jump,
   odom staleness, and `vo_state` as a non-load-bearing bonus. Latched recovery
   (manual `~/reset`); the boot-placeholder covariance is ignored until cuVSLAM first
   converges, so it never false-latches at startup.
2. **`vio_mavros_relay`** — relays cuVSLAM odom to `/mavros/vision_pose/pose`, **gated**
   on `/vio/health`. Degraded → stop publishing → EKF2 times out external vision and
   falls to optical flow. Fail-safe default: no verdict = cut (a dead monitor cuts EV).
3. **`setpoint_bridge` VIO gate** — freezes to a held position and disables follow on a
   degraded verdict, so it never acts on cuVSLAM's drifting pose.

Net behaviour on VIO loss: EKF2 falls to flow → vehicle holds a stable hover → safety
pilot recovers. EGO's local map is invalidated by the dropout, so mission recovery is a
**ground restart**, by design — not an in-flight event.

## Frames

- cuVSLAM owns `odom → camera_link`; PX4 compensates the camera mount via `EKF2_EV_POS`.
- `jetvio_description` adds `base_link` as a child of `camera_link` at the inverse mount
  offset (camera is 0.220 m forward, 0.050 m below the FCU).
- EGO consumes the camera pose in optical-frame convention (`pose_type 1`, ~−90° yaw).
- MAVROS `local_position` reports a large constant z offset (origin-altitude artifact);
  the bridge measures and cancels this constant frame offset at follow-engage.

## Bench vs flight configuration

The tree may be left in **bench** configuration. Before any flight, confirm:

| Parameter | Bench | Flight |
|---|---|---|
| `obstacles_inflation` (ego_live) | 0.2 | **0.75** (hard floor 0.65 = cam→prop-tip; never lower) |
| `z_min` (setpoint_bridge) | −1.0 | **0.3** |
| `bypass_gate` (setpoint_bridge) | true | **false** |
| bounds box `x_max` / `y_max` | 2.0 / 2.0 | **widen to aisle path** (x to aisle length) |

A 0.75 m inflation radius requires ≥ ~1.5 m aisle clear width for EGO to find a path —
confirm aisle geometry before flight; never shrink inflation below 0.65.

## Diagnostics

`pose_snapshot.py` (in `~/jetvio_ws`) captures every pose source at one instant —
cuVSLAM odom, EV relay output, EGO camera pose, MAVROS local position, the bridge
setpoint, vehicle state (read-only), the TF chain, and a z-datum comparison — for
cross-checking transforms. Run with both workspaces sourced.

## Status

**Working / validated:** camera + cuVSLAM VIO, depth pipeline, MAVROS + EKF2 fusion,
EGO live planning, full three-piece VIO-dropout failsafe (detection → EV cut → flow
handoff → bridge hold), perception load-ordering, setpoint-bridge guard layers, and the
transform chain.

**Open items:**
- Tethered Stage 9 flight gate (revert bench config; manual takeoff → follow_enable →
  OFFBOARD → short bounded run → land)
- Confirm H-Flow EKF2 fusion params on PX4; validate flow quality over real floor texture
- Confirm aisle clear width ≥ ~1.5 m against 0.75 inflation
- `jetvio_localization`: cuVSLAM map load for relocalization (scaffold only)
- Flight-test progression: tethered → netted → empty warehouse

## Known benign messages

cuVSLAM small frame-delta warnings; Ctrl-C teardown SIGABRT/SIGSEGV on `ego_planner_node`
(upstream EGO shutdown bug); MAVROS event/time-jump spam; emitter "raw phases barely
differ" on low-contrast scenes.