# ur_state_manager

Manages the state of an **already connected** UR5 (CB3) on the `a200-0553`:
make the arm ready for operation and restore it after a safety violation.

> **Refactor (2026-07):** The former custom mode/safety state machine has been replaced by the
> official **`robot_state_helper`** from the `ur_robot_driver`. This node is now just a **thin
> adapter**: it keeps the familiar `std_srvs/Trigger` API and the node name `ur_state_manager`
> (nothing downstream breaks) and delegates to the `ur_dashboard_msgs/action/SetMode` action of
> the `robot_state_helper`.
>
> The `robot_state_helper` handles `power_on` → `brake_release` → `RUNNING`,
> `unlock_protective_stop`, `restart_safety` on `VIOLATION`/`FAULT`, as well as
> ExternalControl (headless: `resend_robot_program`, otherwise `play`) on its own. This
> package's launch starts it along with the adapter.

It covers two tasks:

1. **Make ready for operation** – `power_on` → `brake_release` → start ExternalControl.
2. **Recovery after a safety violation** – the arm has dropped into a
   lock/stop state after a collision / protective-stop and should be made ready again.

> The node **does not move the arm** and does not send any trajectories. It only ensures
> that the arm is powered, brakes are released, and it is ROS-controllable (ExternalControl active).

## Provided Services (all `std_srvs/Trigger`)

Each service translates into a `SetMode` goal to the `robot_state_helper`:

| Service | Delegates to `SetMode` | Effect |
|---|---|---|
| `~/prepare` | `{RUNNING, play_program}` | Boot up to `RUNNING` + ExternalControl. **Idempotent:** if the arm is already `RUNNING` + safety `NORMAL`/`REDUCED` + ExternalControl active, `prepare` returns `success=True` immediately **without** `robot_state_helper` (the demo also runs through on repeated starts). Otherwise it delegates; `robot_state_helper` skips completed steps on its own. |
| `~/recover` | `[pstop-wait] {RUNNING, stop_program, play_program}` | After a safety violation: stop the program, restore `RUNNING`, restart (UR recommendation after a stop). The helper handles the safety handling. |
| `~/ensure_ready` | like `recover` | `SetMode` does "whatever it takes" anyway → identical to `recover` (including the CB3 wait time). |
| `~/power_off` | `{POWER_OFF, stop_program}` | Safely power off the arm. |

Full service name = node name prefixed, e.g. `/a200_0553/manipulators/ur_state_manager/prepare`.

### Auto-recovery on late arm power-on (`auto_recover`, default on)

If the UR is powered **only after boot**, ExternalControl does not start up: teach pendant
shows "Paused", arm without feedback ("lying down" in RViz), gripper unpowered. A watcher
timer detects the state **"powered, but ExternalControl off"** (`robot_mode` ∈
{`POWER_ON`,`IDLE`,`RUNNING`} and `robot_program_running=False`) and automatically calls
`recover`. Deliberately `recover` (not `prepare`): `stop_program=True` forces a **fresh**
ExternalControl start → the driver syncs `Command=actual` → **no position jump/protective
stop**, unlike a mere `prepare`/`play` that resumes the paused state with a stale command.
Afterwards the `rg6_control` program edge pulls up tool power + prime automatically — late
power-on thus needs **no manual intervention** anymore. Only powered states are touched;
`POWER_OFF`/`BOOTING`/`BACKDRIVE` are left untouched. Disable with `auto_recover:=false`.

> **CB3 special case:** `robot_state_helper` clears a protective stop *immediately*, but the
> CB3 refuses this < 5 s after it was triggered. Therefore `recover`/`ensure_ready` read
> `dashboard_client/get_safety_mode` first and, on `PROTECTIVE_STOP`, wait briefly
> (`protective_stop_wait`, default 6 s) **before** sending the goal.

## Switching controllers per use case (`controller_mode_manager`)

Second node + launch for **switching the arm controllers at runtime**. Idea: ONE
`controller_manager` hosts all controllers; the mutually exclusive **command controllers**
are usually **inactive** and get activated via `switch_controller`. The base set (spawned by
Clearpath: `joint_state_broadcaster`, `arm_0_joint_trajectory_controller`,
`io_and_status_controller`) stays untouched.

`arm_controllers.launch.py` loads the extra controllers `--inactive` (from
`config/extra_controllers.yaml`) and starts the mode manager:

```bash
ros2 launch ur_state_manager arm_controllers.launch.py
```

Modes (default; `mode_names`/`mode_controllers` parameters overridable) — one
`std_srvs/Trigger` service each:

| Service | activates | Purpose |
|---|---|---|
| `~/mode/trajectory` | `arm_0_joint_trajectory_controller` | MoveIt/trajectories (default) |
| `~/mode/freedrive` | `freedrive_mode_controller` | hand-guiding / trajectory recording |
| `~/mode/forward_position` | `forward_position_controller` | direct position streams |
| `~/mode/forward_velocity` | `forward_velocity_controller` | direct velocity streams |
| `~/mode/passthrough` | `passthrough_trajectory_controller` | trajectory streaming |
| `~/release` | – | deactivate all command controllers (arm free) |
| `~/active` | – | report the active command controller |

A switch activates the target controller and **deactivates** the other command controllers
that are currently active (via `switch_controller`, `STRICT`).
Additionally the launch loads the broadcasters `force_torque_sensor_broadcaster`,
`tcp_pose_broadcaster`, `speed_scaling_state_broadcaster` **active** (they do not collide).
The controller parameters in `config/extra_controllers.yaml` are taken 1:1 from
`ur_robot_driver/config/ur_controllers.yaml` (tf_prefix `arm_0_`).

```bash
# e.g. to record a trajectory in FreeDrive
ros2 service call /a200_0553/manipulators/ur_controller_mode_manager/mode/freedrive std_srvs/srv/Trigger
# ... record ... then back:
ros2 service call /a200_0553/manipulators/ur_controller_mode_manager/mode/trajectory std_srvs/srv/Trigger
```

### Recovery logic (`~/recover`)

The complete safety handling now lives in `robot_state_helper` (called from
`recover`/`ensure_ready` via a `SetMode{RUNNING, stop_program, play_program}` goal):

| `safety_mode` | Handling by `robot_state_helper` |
|---|---|
| `NORMAL` / `REDUCED` | No violation → direct mode transition up to `RUNNING`. |
| `PROTECTIVE_STOP` | `unlock_protective_stop` (the **adapter** waits ≥ 6 s beforehand, see above). |
| `SAFEGUARD_STOP` | Cleared by a physical reset; the transition waits for that. |
| `VIOLATION` / `FAULT` | `restart_safety` (arm powers off), then boot up to `RUNNING`. |
| `*_EMERGENCY_STOP` | Not clearable via software → error in the result, physically unlock the E-stop. |

## Prerequisites

- The `ur_robot_driver` is running and connected to the UR5.
- The `io_and_status_controller` is loaded/active (on a200-0553 it is needed for the RG6 anyway).
  The `robot_state_helper` subscribes to `robot_mode`/`safety_mode`/`robot_program_running`
  from it and calls `resend_robot_program`.
- **The `robot_state_helper` node is running.** Clearpath does **not** start it; this launch
  therefore starts it itself (node name `ur_robot_state_helper` in the manipulators namespace).
  It opens its **own** primary-interface connection to `robot_ip:30001` for
  `power_on`/`brake_release`/`unlock_protective_stop`. Check with
  `ros2 action list | grep set_mode` or `ros2 pkg executables ur_robot_driver | grep robot_state_helper`.
- **The `dashboard_client` node is running.** Clearpath does **not** start it in the headless
  setup – `robot_state_helper` needs `restart_safety`/`play` (CB3) from it, the adapter
  `get_safety_mode`. This launch therefore starts it by default
  (`start_dashboard_client:=true`). Alternatively manually:

  ```bash
  ros2 run ur_robot_driver dashboard_client --ros-args \
    -r __ns:=/a200_0553/manipulators \
    -p robot_ip:=192.168.131.40
  ```

  Check with `ros2 service list | grep dashboard`.

  > On a200-0553 the `husky-custom-setup` installer optionally starts the `dashboard_client`
  > as its own boot service (`ur-dashboard.service`). If it is already running through that,
  > start this launch with `start_dashboard_client:=false` so that two dashboard clients do
  > not connect to port 29999 at the same time.
- `headless_mode: true` (Clearpath default on a200-0553) → `robot_state_helper` sends
  ExternalControl via `io_and_status_controller/resend_robot_program`. With
  `headless_mode: false` it uses the dashboard `play` instead.

## Parameters

### `ur_state_manager` (adapter)

| Parameter | Default | Meaning |
|---|---|---|
| `set_mode_action` | `/a200_0553/manipulators/ur_robot_state_helper/set_mode` | Action name of the `robot_state_helper`. |
| `dashboard_ns` | `/a200_0553/manipulators/dashboard_client` | For `get_safety_mode` + `get_robot_mode` (CB3 wait time / idempotent `prepare` pre-check). |
| `io_status_ns` | `/a200_0553/manipulators/io_and_status_controller` | For `robot_program_running` (ExternalControl active? → idempotent `prepare` pre-check). |
| `service_timeout` | `10.0` | Timeout when waiting for action server/service (s). |
| `action_timeout` | `120.0` | Max. wait time for the `SetMode` result (mode transition). |
| `protective_stop_wait` | `6.0` | Wait time before the `SetMode` goal on `PROTECTIVE_STOP` (CB3 ≥ 5 s). |
| `auto_recover` | `true` | Watcher that automatically runs `recover` after late power-on (see above). `false` → off. |
| `auto_recover_period` | `5.0` | Check interval of the watcher (s). |
| `auto_recover_settle` | `2` | This many consistent "must recover" observations before acting (debounces boot/`prepare` transitions). |

### `ur_robot_state_helper` (from `ur_robot_driver`, started by the launch)

| Parameter | Default (launch) | Meaning |
|---|---|---|
| `robot_ip` | `192.168.131.40` | UR control box (primary interface port 30001). |
| `headless_mode` | `true` | `true` → ExternalControl via `resend_robot_program`, otherwise `play`. |

## Build

```bash
git clone https://github.com/CLAIRLab-HAW/ur-state-manager.git
cd ur-state-manager
colcon build --packages-select ur_state_manager
source install/setup.bash
```

`ur_dashboard_msgs` comes with the `ur_robot_driver` stack (present on the a200-0553).

> On a200-0553 the `husky-custom-setup` installer optionally does this automatically:
> it clones+builds this repo and installs `ur-state-manager.service` (starts the
> manager at boot, `start_dashboard_client:=false`, since the `dashboard_client`
> runs via `ur-dashboard.service`).

## Starting

```bash
# starts dashboard_client + robot_state_helper + ur_state_manager (adapter)
# robot_ip 192.168.131.40, headless_mode:=true
ros2 launch ur_state_manager ur_state_manager.launch.py

# if the dashboard_client is already running elsewhere:
ros2 launch ur_state_manager ur_state_manager.launch.py start_dashboard_client:=false

# or just the adapter directly (requires robot_state_helper to already be running):
ros2 run ur_state_manager state_manager --ros-args \
  -r __ns:=/a200_0553/manipulators \
  -p set_mode_action:=/a200_0553/manipulators/ur_robot_state_helper/set_mode \
  -p dashboard_ns:=/a200_0553/manipulators/dashboard_client
```

## Using

```bash
# make the arm ready for operation (power on + release brakes + ExternalControl)
ros2 service call /a200_0553/manipulators/ur_state_manager/prepare std_srvs/srv/Trigger

# make ready again after a collision / protective-stop
ros2 service call /a200_0553/manipulators/ur_state_manager/recover std_srvs/srv/Trigger

# "just make it ready, regardless of state"
ros2 service call /a200_0553/manipulators/ur_state_manager/ensure_ready std_srvs/srv/Trigger

# safely power off
ros2 service call /a200_0553/manipulators/ur_state_manager/power_off std_srvs/srv/Trigger
```

Each response returns `success` (bool) and `message` (string) with a plain-text status.
Only **one** operation runs at a time (parallel calls are rejected with
`success=false`).

## Note on integration via `robot.yaml`

For the workspace to be found, it must — like `rg6_control` — be listed under
`system.ros2.workspaces` in the `robot.yaml`. The node can then be started alongside
`rg6_bringup.launch.py` via `platform.extras.launch`.