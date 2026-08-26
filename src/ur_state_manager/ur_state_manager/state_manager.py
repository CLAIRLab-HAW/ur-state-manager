#!/usr/bin/env python3
"""Thin adapter onto the official 'robot_state_helper' (ur_robot_driver).

This node is an *adapter*: it keeps the familiar std_srvs/Trigger API (prepare / recover / ensure_ready / power_off)
and the node name 'ur_state_manager', so that existing callers keep running unchanged, and delegates the work to the
SetMode action of the robot_state_helper.

What robot_state_helper does itself and is NOT rebuilt here: power_on ─▶ brake_release ─▶ RUNNING,
unlock_protective_stop on PROTECTIVE_STOP, restart_safety on VIOLATION/FAULT, (re)starting ExternalControl.  An E-stop
is only reported, never released by software.

The only ingredient robot_state_helper does NOT know about: the CB3 requirement to wait >=5 s after a protective stop
before unlock_protective_stop is accepted. It unlocks immediately ─▶ on the CB3 that can fail.  That is why
'recover'/'ensure_ready' read the safety_mode beforehand and wait briefly if needed, BEFORE the SetMode goal goes out.

Four ingredients against the restart / first-power-up traps (a200-0553):

* VERIFICATION + RETRY: on the FIRST brake release after a longer period
  without power, the CB3 frequently throws a protective stop or FAULT out of
  its own start-up procedure, BEFORE ROS streams anything; the second attempt
  gets through reliably.  robot_state_helper notices none of this -- its goal
  reports success as soon as RUNNING is reached and play/resend has been sent,
  and the p-stop falls into the gap in between.  That is why this adapter
  checks for itself after every SetMode (RUNNING + safety NORMAL/REDUCED +
  ExternalControl running) and repeats the bring-up (bringup_attempts,
  default 3).
* HELPER PRIMING: robot_state_helper subscribes to robot_mode/safety_mode
  BEST_EFFORT+VOLATILE; the GPIOController publishes TRANSIENT_LOCAL and only
  ON CHANGE.  After a restart of this service alone the helper stays blind
  ("Robot mode is unknown") and rejects every goal -- nobody republishes,
  after all.  Before every goal this adapter publishes the current state read
  via the dashboard ONCE onto the same topics.
* CONTROLLER RELEASE before the mode cycle: after a manipulators restart the
  trajectory controller is active, but its hold target dates from BEFORE the
  power-up.  If ExternalControl starts with that stale hold value, the driver
  streams straight there ─▶ position jump.  A release before the bring-up +
  a fresh activation afterwards closes the gap.
* LATCHED QoS + DASHBOARD FALLBACK: robot_program_running arrives
  TRANSIENT_LOCAL and only on change ─▶ the subscription here is likewise.
  Under rmw_zenoh the latched value nevertheless does NOT reliably reach late
  joiners ─▶ as long as the topic has delivered nothing, the dashboard server
  steps in.  Without both, pre-check, verification and auto recovery would be
  blind after every adapter restart.

Mapping of the Trigger services onto SetMode goals:
  ~/prepare       [idempotent] SetMode{RUNNING, stop_program=false, play_program=true}
                  pre-check: already RUNNING + ExternalControl + safety ok ─▶
                  success=True WITHOUT robot_state_helper.  Retries run as
                  recover (stop_program=true, clean restart).
  ~/recover       [pstop-wait] SetMode{RUNNING, stop_program=true, play_program=true}
  ~/ensure_ready  like recover
  ~/power_off     SetMode{POWER_OFF, stop_program=true, play_program=false}

All names are parameters (the defaults fit the a200-0553).
"""

import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from ur_dashboard_msgs.action import SetMode
from ur_dashboard_msgs.msg import RobotMode, SafetyMode
from ur_dashboard_msgs.srv import GetRobotMode, GetSafetyMode, IsProgramRunning

# Human-readable names for log output (the constants come from the .msg files).
ROBOT_MODE_NAMES = {
    RobotMode.NO_CONTROLLER: "NO_CONTROLLER",
    RobotMode.DISCONNECTED: "DISCONNECTED",
    RobotMode.CONFIRM_SAFETY: "CONFIRM_SAFETY",
    RobotMode.BOOTING: "BOOTING",
    RobotMode.POWER_OFF: "POWER_OFF",
    RobotMode.POWER_ON: "POWER_ON",
    RobotMode.IDLE: "IDLE",
    RobotMode.BACKDRIVE: "BACKDRIVE",
    RobotMode.RUNNING: "RUNNING",
    RobotMode.UPDATING_FIRMWARE: "UPDATING_FIRMWARE",
}

SAFETY_MODE_NAMES = {
    SafetyMode.NORMAL: "NORMAL",
    SafetyMode.REDUCED: "REDUCED",
    SafetyMode.PROTECTIVE_STOP: "PROTECTIVE_STOP",
    SafetyMode.RECOVERY: "RECOVERY",
    SafetyMode.SAFEGUARD_STOP: "SAFEGUARD_STOP",
    SafetyMode.SYSTEM_EMERGENCY_STOP: "SYSTEM_EMERGENCY_STOP",
    SafetyMode.ROBOT_EMERGENCY_STOP: "ROBOT_EMERGENCY_STOP",
    SafetyMode.VIOLATION: "VIOLATION",
    SafetyMode.FAULT: "FAULT",
    SafetyMode.VALIDATE_JOINT_ID: "VALIDATE_JOINT_ID",
    SafetyMode.UNDEFINED_SAFETY_MODE: "UNDEFINED_SAFETY_MODE",
}


def _robot_mode_name(mode):
    return ROBOT_MODE_NAMES.get(mode, f"UNKNOWN({mode})")


def _safety_mode_name(mode):
    return SAFETY_MODE_NAMES.get(mode, f"UNKNOWN({mode})")


class StateManager(Node):
    def __init__(self):
        super().__init__("ur_state_manager")

        # ---- parameters ---------------------------------------------------
        # Action of the robot_state_helper. It runs (see the launch file) as
        # node 'ur_robot_state_helper' in the manipulators namespace.
        self.set_mode_action = self.declare_parameter(
            "set_mode_action", "/a200_0553/manipulators/ur_robot_state_helper/set_mode"
        ).value
        # Only needed for the CB3 wait before the (internally immediate) unlock; additionally for the idempotent
        # prepare pre-check (get_robot_mode).
        dashboard_ns = self.declare_parameter("dashboard_ns", "/a200_0553/manipulators/dashboard_client").value.rstrip(
            "/"
        )
        # io_and_status_controller: supplies robot_program_running (is ExternalControl active?) for the idempotent
        # prepare pre-check.
        io_status_ns = self.declare_parameter(
            "io_status_ns", "/a200_0553/manipulators/io_and_status_controller"
        ).value.rstrip("/")
        # controller_mode_manager: after a successful prepare/recover the trajectory mode is activated as well.  A
        # power_off stops the ExternalControl program ─▶ the driver reports its command interfaces as unavailable ─▶
        # ros2_control MUST deactivate every controller that claims them.  On the way back up ros2_control does NOT
        # activate it again by itself: without this step the arm stays powered and connected, but every MoveIt
        # execution fails -- without the error message pointing at the inactive controller.
        mode_manager_ns = self.declare_parameter(
            "mode_manager_ns", "/a200_0553/manipulators/ur_controller_mode_manager"
        ).value.rstrip("/")
        self.trajectory_mode = self.declare_parameter("trajectory_mode", "trajectory").value
        self.ensure_trajectory_mode = bool(self.declare_parameter("ensure_trajectory_mode", True).value)

        self.service_timeout = float(self.declare_parameter("service_timeout", 10.0).value)
        # How long a mode transition (e.g. POWER_OFF ─▶ RUNNING) may take.
        self.action_timeout = float(self.declare_parameter("action_timeout", 120.0).value)
        # The CB3 refuses to release a protective stop < 5 s after it triggered.
        self.protective_stop_wait = float(self.declare_parameter("protective_stop_wait", 6.0).value)
        # After a "successful" SetMode: this is how long it may take until the arm is REALLY ready (RUNNING + safety
        # NORMAL/REDUCED + ExternalControl running). A protective stop/FAULT aborts the wait immediately.
        self.verify_ready_timeout = float(self.declare_parameter("verify_ready_timeout", 20.0).value)
        # Bring-up attempts in total. The CB3 brake-release p-stop (module docstring) empirically always heals on the
        # second attempt; 3 leaves room for a FAULT (restart_safety) in between.
        self.bringup_attempts = int(self.declare_parameter("bringup_attempts", 3).value)
        # Deactivate the command controllers (JTC & co.) before every mode cycle.
        self.release_before_power_cycle = bool(self.declare_parameter("release_before_power_cycle", True).value)

        # Clients + servers in one ReentrantCallbackGroup, so that we can await the action synchronously from inside a
        # service callback (the response is processed by another thread of the MultiThreadedExecutor).
        self.cbg = ReentrantCallbackGroup()

        self.cli_set_mode = ActionClient(self, SetMode, self.set_mode_action, callback_group=self.cbg)
        self.cli_get_safety_mode = self.create_client(
            GetSafetyMode, f"{dashboard_ns}/get_safety_mode", callback_group=self.cbg
        )
        self.cli_get_robot_mode = self.create_client(
            GetRobotMode, f"{dashboard_ns}/get_robot_mode", callback_group=self.cbg
        )
        self.cli_program_running = self.create_client(
            IsProgramRunning, f"{dashboard_ns}/program_running", callback_group=self.cbg
        )
        self.cli_trajectory_mode = self.create_client(
            Trigger, f"{mode_manager_ns}/mode/{self.trajectory_mode}", callback_group=self.cbg
        )
        self.cli_release = self.create_client(Trigger, f"{mode_manager_ns}/release", callback_group=self.cbg)

        # Priming publishers (module docstring): a one-off publication of the current state onto the GPIOController
        # topics, so that the BEST_EFFORT/VOLATILE subscriber in the robot_state_helper does not stay blind after a
        # (partial) restart. Deliberately VOLATILE: NOTHING may latch that later lands as a stale sample with a late
        # joiner - the GPIOController remains the owner of the topics.
        self.pub_robot_mode = self.create_publisher(RobotMode, f"{io_status_ns}/robot_mode", 1)
        self.pub_safety_mode = self.create_publisher(SafetyMode, f"{io_status_ns}/safety_mode", 1)

        # ExternalControl status (True = the ROS program is running) for the idempotent prepare pre-check and the
        # bring-up verification. The GPIOController publishes TRANSIENT_LOCAL and ONLY on change ─▶ the subscription is
        # TRANSIENT_LOCAL as well, otherwise the value stays None after an adapter restart until the program changes
        # the next time (pre-check and watcher blind).
        self._program_running = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            Bool, f"{io_status_ns}/robot_program_running", self._on_program_running, latched, callback_group=self.cbg
        )

        # ---- Own services (unchanged from the old API) -----------------
        self._lock = threading.Lock()  # never two processes at the same time
        self.create_service(Trigger, "~/prepare", self._srv_prepare, callback_group=self.cbg)
        self.create_service(Trigger, "~/recover", self._srv_recover, callback_group=self.cbg)
        self.create_service(Trigger, "~/ensure_ready", self._srv_ensure_ready, callback_group=self.cbg)
        self.create_service(Trigger, "~/power_off", self._srv_power_off, callback_group=self.cbg)

        # ---- auto-recovery watcher (arm powered up late) -------------------
        # If the UR is powered only AFTER the boot, ExternalControl does not
        # start (teach pendant "Paused", arm without feedback).  This watcher
        # recognizes "powered, but ExternalControl off" and calls recover on
        # its own.  recover uses stop_program=True (clean restart ─▶ the driver
        # syncs command=actual ─▶ NO position jump, unlike a bare prepare/play,
        # which continues the paused state with a stale command).  The gripper
        # is not affected: it hangs off the OnRobot URCap, not off the tool
        # connector.  auto_recover=false switches the automation off.
        self.auto_recover = bool(self.declare_parameter("auto_recover", True).value)
        self.auto_recover_period = float(self.declare_parameter("auto_recover_period", 5.0).value)
        # this many consecutive "needs recovery" observations before acting (debounces boot/prepare transitions in
        # which the state matches briefly).
        self.auto_recover_settle = int(self.declare_parameter("auto_recover_settle", 2).value)
        self._needs_recover_count = 0
        if self.auto_recover:
            self.create_timer(self.auto_recover_period, self._auto_recover_tick, callback_group=self.cbg)

        self.get_logger().info(
            f"ur_state_manager (adapter) ready. set_mode_action={self.set_mode_action} "
            f"dashboard_ns={dashboard_ns} auto_recover={self.auto_recover}"
        )

    # ======================================================================
    # low-level helpers
    # ======================================================================
    def _spin_future(self, future, timeout):
        """Wait for a ``*_async`` future without blocking the executor thread."""
        done = threading.Event()
        future.add_done_callback(lambda _f: done.set())
        return done.wait(timeout) and future.done()

    def _sleep(self, seconds):
        """Non-blocking wait (releases the thread)."""
        threading.Event().wait(seconds)

    def _on_feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f"SetMode feedback: robot_mode={_robot_mode_name(fb.current_robot_mode)} "
            f"safety_mode={_safety_mode_name(fb.current_safety_mode)}"
        )

    def _on_program_running(self, msg: Bool):
        self._program_running = bool(msg.data)

    def _get_safety_mode(self):
        """Read safety_mode via the dashboard client. ─▶ mode | None."""
        if not self.cli_get_safety_mode.wait_for_service(timeout_sec=self.service_timeout):
            return None
        fut = self.cli_get_safety_mode.call_async(GetSafetyMode.Request())
        if not self._spin_future(fut, self.service_timeout):
            return None
        return fut.result().safety_mode.mode

    def _get_robot_mode(self):
        """Read robot_mode via the dashboard client. ─▶ mode | None."""
        if not self.cli_get_robot_mode.wait_for_service(timeout_sec=self.service_timeout):
            return None
        fut = self.cli_get_robot_mode.call_async(GetRobotMode.Request())
        if not self._spin_future(fut, self.service_timeout):
            return None
        return fut.result().robot_mode.mode

    def _effective_program_running(self):
        """ExternalControl status: topic value, otherwise dashboard fallback. ─▶ bool | None.

        Under rmw_zenoh the latched robot_program_running value does NOT reliably reach a late joiner (only live
        changes; measured empirically on the a200-0553). So as long as the topic has delivered nothing yet, this
        fallback asks the dashboard server ('program_running') - in headless operation the ExternalControl script runs
        there as the program."""
        if self._program_running is not None:
            return self._program_running
        if not self.cli_program_running.wait_for_service(timeout_sec=self.service_timeout):
            return None
        fut = self.cli_program_running.call_async(IsProgramRunning.Request())
        if not self._spin_future(fut, self.service_timeout):
            return None
        res = fut.result()
        if not res.success:
            return None
        return bool(res.program_running)

    def _already_ready(self):
        """Idempotence check for prepare: is the arm already in service
        (RUNNING + safety NORMAL/REDUCED + ExternalControl active), so that NO
        mode change and hence no robot_state_helper is needed? ─▶ bool."""
        robot_mode = self._get_robot_mode()
        safety = self._get_safety_mode()
        prog = self._effective_program_running()
        if robot_mode == RobotMode.RUNNING and safety in (SafetyMode.NORMAL, SafetyMode.REDUCED) and prog is True:
            self.get_logger().info(
                "prepare: arm already RUNNING + ExternalControl active "
                "─▶ no mode change needed (robot_state_helper not required)."
            )
            return True
        self.get_logger().info(
            "prepare: not ready straight away (robot_mode="
            f"{_robot_mode_name(robot_mode) if robot_mode is not None else 'unknown'}, "
            f"safety={_safety_mode_name(safety) if safety is not None else 'unknown'}, "
            f"program_running={prog}) ─▶ delegating to robot_state_helper."
        )
        return False

    def _wait_if_protective_stop(self):
        """CB3: wait >=5 s after a protective stop before the robot_state_helper unlocks."""
        safety = self._get_safety_mode()
        if safety == SafetyMode.PROTECTIVE_STOP:
            self.get_logger().info(
                f"protective stop detected ─▶ waiting {self.protective_stop_wait}s "
                "(CB3 requirement) before the unlock ..."
            )
            self._sleep(self.protective_stop_wait)
        elif safety is None:
            self.get_logger().warn(
                "safety_mode not readable (is the dashboard client there?) - continuing "
                "without the CB3 waiting time; call recover again if needed."
            )

    def _prime_state_helper(self):
        """Make the robot_state_helper 'seeing' before every goal (module docstring).

        Read the current state via the dashboard and publish it ONCE onto the robot_mode/safety_mode topics. Without
        that, after a restart of this service (without a driver restart) the helper rejects every goal, because it has
        missed the latched values of the GPIOController and nobody republishes."""
        robot_mode = self._get_robot_mode()
        safety = self._get_safety_mode()
        if robot_mode is None and safety is None:
            self.get_logger().warn(
                "priming skipped: the dashboard supplies neither robot_mode nor "
                "safety_mode - the goal may fail on the helper's 'unknown mode' "
                "check."
            )
            return
        if robot_mode is not None:
            self.pub_robot_mode.publish(RobotMode(mode=int(robot_mode)))
        if safety is not None:
            self.pub_safety_mode.publish(SafetyMode(mode=int(safety)))
        # give the helper a moment to process the samples before the goal
        self._sleep(0.3)

    def _set_mode(self, target, stop_program, play_program):
        """Send a SetMode goal and wait synchronously for the result. ─▶ (ok, msg)."""
        if not self.cli_set_mode.wait_for_server(timeout_sec=self.service_timeout):
            return False, (
                "robot_state_helper/set_mode action not available - " "is the ur_robot_state_helper node running?"
            )
        self._prime_state_helper()

        goal = SetMode.Goal()
        goal.target_robot_mode = target
        goal.stop_program = stop_program
        goal.play_program = play_program
        self.get_logger().info(
            f"SetMode ─▶ target={_robot_mode_name(target)} " f"stop_program={stop_program} play_program={play_program}"
        )

        send_fut = self.cli_set_mode.send_goal_async(goal, feedback_callback=self._on_feedback)
        if not self._spin_future(send_fut, self.service_timeout):
            return False, "SetMode: timeout while sending the goal"
        handle = send_fut.result()
        if not handle.accepted:
            # Upstream (jazzy) rejects ONLY when robot_mode/safety_mode are still UNKNOWN/UNDEFINED (the helper has no
            # status data yet from the freshly started driver; under rmw_zenoh discovery takes a few seconds). There is
            # no busy check upstream - competing goals are accepted.
            return False, (
                "SetMode goal rejected - robot_state_helper presumably not "
                "ready yet (robot_mode/safety_mode not received yet, e.g. "
                "right after a stack restart); the next attempt usually "
                "heals it."
            )

        res_fut = handle.get_result_async()
        if not self._spin_future(res_fut, self.action_timeout):
            return (False, f"SetMode: timeout ({self.action_timeout}s) while waiting for the result")
        result = res_fut.result().result
        return result.success, result.message

    # ======================================================================
    # sequences (delegating to robot_state_helper)
    # ======================================================================
    def _ensure_trajectory_mode(self):
        """Activate the trajectory controller (best effort, never fatal).

        Called after a successful prepare/recover. Idempotent: the controller_mode_manager only switches when needed.
        If it fails (mode manager absent, timeout), it only warns - the arm is then powered and connected, only the
        controller is missing; that is better than a prepare counted as an error because of it."""
        if not self.ensure_trajectory_mode:
            return
        if not self.cli_trajectory_mode.wait_for_service(timeout_sec=self.service_timeout):
            self.get_logger().warn(
                f"trajectory mode: {self.cli_trajectory_mode.srv_name} not "
                "reachable (is the controller_mode_manager running?) - the arm is "
                "ready, but MoveIt execution fails until the "
                "arm_0_joint_trajectory_controller is active."
            )
            return
        fut = self.cli_trajectory_mode.call_async(Trigger.Request())
        if not self._spin_future(fut, self.service_timeout):
            self.get_logger().warn("trajectory mode: timeout while switching over.")
            return
        res = fut.result()
        if res.success:
            self.get_logger().info(f"trajectory mode active ({res.message}).")
        else:
            self.get_logger().warn(f"trajectory mode not set: {res.message}")

    def _release_command_controllers(self):
        """Deactivate the command controllers before a mode cycle (best effort).

        After a manipulators restart the arm_0_joint_trajectory_controller is active, but its hold target dates from
        BEFORE the power-up (brakes closed). If ExternalControl starts with that stale hold value, the driver streams
        straight there ─▶ position jump / following error. A release here + a fresh activation in
        _ensure_trajectory_mode AFTER the bring-up closes the gap. Never fatal: without the mode manager the bring-up
        runs as before."""
        if not self.release_before_power_cycle:
            return
        if not self.cli_release.wait_for_service(timeout_sec=self.service_timeout):
            self.get_logger().warn(
                f"controller release: {self.cli_release.srv_name} not reachable "
                "(is the controller_mode_manager running?) - continuing without the release."
            )
            return
        fut = self.cli_release.call_async(Trigger.Request())
        if not self._spin_future(fut, self.service_timeout):
            self.get_logger().warn("controller release: timeout - continuing.")
            return
        res = fut.result()
        log = self.get_logger().info if res.success else self.get_logger().warn
        log(f"controller release before the mode cycle: {res.message}")

    _GOOD_SAFETY = (SafetyMode.NORMAL, SafetyMode.REDUCED)
    _TERMINAL_SAFETY = (SafetyMode.SYSTEM_EMERGENCY_STOP, SafetyMode.ROBOT_EMERGENCY_STOP)

    def _verify_ready(self, timeout):
        """After a 'successful' SetMode, check whether the arm is REALLY ready.

        robot_state_helper reports success as soon as RUNNING is reached and play/resend has been sent - a protective
        stop that falls DURING the bring-up (CB3 brake release, module docstring) slips through that gap. Here:
        RUNNING + safety NORMAL/REDUCED + ExternalControl running, polled until ``timeout``; a p-stop/FAULT/VIOLATION
        aborts immediately (a retry heals it), an E-stop aborts for good. ─▶ (ok, detail, retryable)."""
        deadline = time.monotonic() + max(0.0, timeout)
        while True:
            robot_mode = self._get_robot_mode()
            safety = self._get_safety_mode()
            prog = self._effective_program_running()
            if robot_mode == RobotMode.RUNNING and safety in self._GOOD_SAFETY and prog is True:
                return True, "", True
            detail = (
                "robot_mode="
                f"{_robot_mode_name(robot_mode) if robot_mode is not None else 'unknown'} "
                f"safety={_safety_mode_name(safety) if safety is not None else 'unknown'} "
                f"program_running={prog}"
            )
            if safety in self._TERMINAL_SAFETY:
                return False, f"{detail} (E-stop: can only be released manually)", False
            if safety in (SafetyMode.PROTECTIVE_STOP, SafetyMode.VIOLATION, SafetyMode.FAULT):
                return False, detail, True
            if time.monotonic() >= deadline:
                return False, detail, True
            self._sleep(0.5)

    def _bringup(self, stop_program_first):
        """Bring-up with verification + retry (the core of prepare/recover).

        One attempt: [CB3 p-stop wait] ─▶ controller release ─▶ SetMode(RUNNING)
        ─▶ verification. The CB3 brake-release p-stop of the first attempt
        (module docstring) is thus healed INSIDE a single Trigger call, instead
        of leaving the caller a false success (or a half-dead arm). From the
        second attempt on, always stop_program=True (clean program restart, UR's
        recommendation after every stop)."""
        attempts = max(1, self.bringup_attempts)
        msg = ""
        for attempt in range(1, attempts + 1):
            if attempt > 1:
                self.get_logger().warn(f"bring-up attempt {attempt}/{attempts} (previously: {msg})")
            self._wait_if_protective_stop()
            self._release_command_controllers()
            ok, msg = self._set_mode(
                RobotMode.RUNNING, stop_program=stop_program_first or attempt > 1, play_program=True
            )
            if not ok:
                continue
            ok, detail, retryable = self._verify_ready(self.verify_ready_timeout)
            if ok:
                self._ensure_trajectory_mode()
                if attempt > 1:
                    return True, f"ready (attempt {attempt}/{attempts})"
                return True, (msg or "ready")
            msg = f"bring-up not verified: {detail}"
            if not retryable:
                break
        return (False, f"arm not ready after {attempts} attempts - last state: {msg}")

    def prepare(self):
        """Arm in service: RUNNING + ExternalControl + trajectory controller.

        Idempotent: if the arm is already in service, success=True is reported at once, WITHOUT the robot_state_helper
        -- this way the demo also gets through on a repeated start, even if the helper happens to be unreachable.

        The trajectory mode is ensured in BOTH cases, in the idempotence branch too: after a power_off the arm is
        quickly RUNNING again, but the controller is deactivated.
        """
        if self._already_ready():
            self._ensure_trajectory_mode()
            return True, "already in service (RUNNING, ExternalControl active)"
        return self._bringup(stop_program_first=False)

    def recover(self):
        """Ready again after a safety violation: stop the program, RUNNING, restart it.

        robot_state_helper handles PROTECTIVE_STOP / VIOLATION / FAULT / E-stop itself; beforehand we only sit out the
        mandatory CB3 waiting time. stop_program=true matches UR's recommendation to start the program ANEW after a
        stop (instead of simply resuming it).
        """
        return self._bringup(stop_program_first=True)

    def power_off(self):
        """Power the arm down safely. Release the controllers first: on a program
        stop the driver reports its command interfaces as unavailable anyway, and
        the release turns that into an orderly step instead of a forced
        deactivation (and holds no stale hold target for the next start)."""
        self._release_command_controllers()
        return self._set_mode(RobotMode.POWER_OFF, stop_program=True, play_program=False)

    # ======================================================================
    # auto recovery: brings the arm up after a late power-up with no manual step
    # ======================================================================
    def _needs_recover(self):
        """True when the arm is powered but ExternalControl is NOT running.

        Exactly the state after a late power-up / 'Paused': robot_mode in {POWER_ON, IDLE, RUNNING}, but
        robot_program_running=False. POWER_OFF / DISCONNECTED / BOOTING (arm deliberately off, or still booting) and
        BACKDRIVE (freedrive) are NOT touched. An unknown program status (None, even after the dashboard fallback) ─▶
        do not act (the safe default)."""
        if self._effective_program_running() is not False:
            return False  # already running, or the status is still unknown
        mode = self._get_robot_mode()
        return mode in (RobotMode.POWER_ON, RobotMode.IDLE, RobotMode.RUNNING)

    def _auto_recover_tick(self):
        # Is a prepare/recover already running (manual OR automatic)? ─▶ do not butt in.
        if self._lock.locked():
            self._needs_recover_count = 0
            return
        if not self._needs_recover():
            self._needs_recover_count = 0
            return
        self._needs_recover_count += 1
        if self._needs_recover_count < max(1, self.auto_recover_settle):
            return  # debounce: only act after several consistent observations
        self._needs_recover_count = 0
        self.get_logger().warn(
            "auto recovery: arm powered, but ExternalControl is not running "
            "(late power-up / Paused) ─▶ running recover ..."
        )
        resp = Trigger.Response()
        self._run_locked(self.recover, resp)
        self.get_logger().info(f"auto recovery: recover ─▶ success={resp.success} ({resp.message})")

    # ======================================================================
    # service callbacks
    # ======================================================================
    def _run_locked(self, fn, response):
        if not self._lock.acquire(blocking=False):
            response.success = False
            response.message = "a prepare/recover process is already underway"
            return response
        try:
            ok, msg = fn()
            response.success = ok
            response.message = msg
        except Exception as exc:  # defensive: never let the service thread die
            self.get_logger().error(f"Exception: {exc}")
            response.success = False
            response.message = f"Exception: {exc}"
        finally:
            self._lock.release()
        return response

    def _srv_prepare(self, _request, response):
        return self._run_locked(self.prepare, response)

    def _srv_recover(self, _request, response):
        return self._run_locked(self.recover, response)

    def _srv_ensure_ready(self, _request, response):
        # SetMode does "whatever is needed" anyway ─▶ identical to recover (including the CB3 wait).
        return self._run_locked(self.recover, response)

    def _srv_power_off(self, _request, response):
        return self._run_locked(self.power_off, response)


def main():
    rclpy.init()
    node = StateManager()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
