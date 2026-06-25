#!/usr/bin/env python3
"""ROS2-Node zum Verwalten eines bereits verbundenen UR5 (CB3) auf der a200-0553.

Der Node spricht ueber den 'ur_robot_driver' Dashboard-Client (Port 29999 auf der
UR-Control-Box) sowie ueber den 'io_and_status_controller' und bietet vier eigene
std_srvs/Trigger-Services an:

  ~/prepare       Arm einsatzbereit machen: power_on + brake_release + ExternalControl
                  (im headless_mode via resend_robot_program, sonst load+play).
  ~/recover       Nach einer Safety-Violation wieder bereit machen:
                  je nach safety_mode unlock_protective_stop / restart_safety,
                  danach automatisch erneut prepare().
  ~/ensure_ready  Komfort: liegt eine Safety-Violation vor -> recover, sonst prepare.
  ~/power_off     Arm sicher abschalten (power_off).

Hintergrund a200-0553 (siehe Projekt-Memory):
  * UR5 CB3, Clearpath startet den Treiber mit 'headless_mode: true' -> das
    ExternalControl-Programm wird vom Treiber direkt gesendet, NICHT von einer
    laufenden URCap. Nach Power-Cycle / Protective-Stop muss die Kontrolle daher
    ueber 'io_and_status_controller/resend_robot_program' neu gesendet werden.
  * Der RG6-Greifer braucht den io_and_status_controller ohnehin; dieser Node
    haengt aber NICHT vom Greifer ab.

Alle Service-/Namespace-Pfade sind Parameter (Defaults passen zu a200-0553).
"""

import threading

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from std_srvs.srv import Trigger
from ur_dashboard_msgs.msg import RobotMode, SafetyMode
from ur_dashboard_msgs.srv import GetRobotMode, GetSafetyMode, Load


# Menschenlesbare Namen fuer Logausgaben (Konstanten kommen aus den .msg).
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

        # ---- Parameter ----------------------------------------------------
        ns = self.declare_parameter("dashboard_ns",
                                    "/a200_0553/manipulators/dashboard_client").value
        io_ns = self.declare_parameter("io_status_ns",
                                       "/a200_0553/manipulators/io_and_status_controller").value
        self.headless_mode = self.declare_parameter("headless_mode", True).value
        # Nur fuer headless_mode=False relevant: zu ladendes .urp-Programm vor 'play'.
        self.program_name = self.declare_parameter("program_name", "").value

        self.service_timeout = float(self.declare_parameter("service_timeout", 10.0).value)
        # CB3 verweigert das Loesen eines Protective-Stops < 5 s nach dem Ausloesen.
        self.protective_stop_wait = float(self.declare_parameter("protective_stop_wait", 6.0).value)
        # Wie lange wir auf einen Mode-Wechsel (z.B. -> RUNNING) warten.
        self.mode_timeout = float(self.declare_parameter("mode_timeout", 30.0).value)
        self.mode_poll = float(self.declare_parameter("mode_poll_interval", 0.5).value)

        ns = ns.rstrip("/")
        io_ns = io_ns.rstrip("/")

        # Damit Service-Server-Callbacks waehrend laufender Dashboard-Calls nicht
        # blockieren (wir rufen synchron aus dem Callback heraus), liegen alle
        # Clients UND Server in einer ReentrantCallbackGroup + MultiThreadedExecutor.
        self.cbg = ReentrantCallbackGroup()

        # ---- Dashboard-/IO-Clients ---------------------------------------
        def trig(name):
            return self.create_client(Trigger, name, callback_group=self.cbg)

        self.cli_power_on = trig(f"{ns}/power_on")
        self.cli_power_off = trig(f"{ns}/power_off")
        self.cli_brake_release = trig(f"{ns}/brake_release")
        self.cli_unlock_pstop = trig(f"{ns}/unlock_protective_stop")
        self.cli_restart_safety = trig(f"{ns}/restart_safety")
        self.cli_close_safety_popup = trig(f"{ns}/close_safety_popup")
        self.cli_close_popup = trig(f"{ns}/close_popup")
        self.cli_play = trig(f"{ns}/play")
        self.cli_get_robot_mode = self.create_client(
            GetRobotMode, f"{ns}/get_robot_mode", callback_group=self.cbg)
        self.cli_get_safety_mode = self.create_client(
            GetSafetyMode, f"{ns}/get_safety_mode", callback_group=self.cbg)
        self.cli_load_program = self.create_client(
            Load, f"{ns}/load_program", callback_group=self.cbg)

        # ExternalControl (re)starten -> liefert der io_and_status_controller.
        self.cli_resend_program = trig(f"{io_ns}/resend_robot_program")

        # ---- Eigene Services ---------------------------------------------
        # Eine globale Sperre: nie zwei Ablaeufe gleichzeitig.
        self._lock = threading.Lock()
        self.create_service(Trigger, "~/prepare", self._srv_prepare, callback_group=self.cbg)
        self.create_service(Trigger, "~/recover", self._srv_recover, callback_group=self.cbg)
        self.create_service(Trigger, "~/ensure_ready", self._srv_ensure_ready, callback_group=self.cbg)
        self.create_service(Trigger, "~/power_off", self._srv_power_off, callback_group=self.cbg)

        self.get_logger().info(
            f"ur_state_manager bereit. dashboard_ns={ns} io_status_ns={io_ns} "
            f"headless_mode={self.headless_mode}")

    # ======================================================================
    # Low-Level-Helfer
    # ======================================================================
    def _spin_future(self, future, timeout):
        """Auf ein call_async-Future warten, ohne den Executor-Thread zu blockieren.

        Funktioniert, weil die Future-Antwort von einem anderen Thread des
        MultiThreadedExecutor verarbeitet wird (Clients in ReentrantCallbackGroup).
        """
        done = threading.Event()
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout):
            return False
        return future.done()

    def _trigger(self, client, label):
        """std_srvs/Trigger-Service aufrufen -> (success, message)."""
        if not client.wait_for_service(timeout_sec=self.service_timeout):
            return False, f"Service '{label}' nicht verfuegbar"
        fut = client.call_async(Trigger.Request())
        if not self._spin_future(fut, self.service_timeout):
            return False, f"'{label}' Timeout"
        res = fut.result()
        self.get_logger().info(f"{label}: success={res.success} msg=\"{res.message}\"")
        return res.success, res.message

    def _get_robot_mode(self):
        if not self.cli_get_robot_mode.wait_for_service(timeout_sec=self.service_timeout):
            return None
        fut = self.cli_get_robot_mode.call_async(GetRobotMode.Request())
        if not self._spin_future(fut, self.service_timeout):
            return None
        return fut.result().robot_mode.mode

    def _get_safety_mode(self):
        if not self.cli_get_safety_mode.wait_for_service(timeout_sec=self.service_timeout):
            return None
        fut = self.cli_get_safety_mode.call_async(GetSafetyMode.Request())
        if not self._spin_future(fut, self.service_timeout):
            return None
        return fut.result().safety_mode.mode

    def _wait_for_robot_mode(self, targets, timeout=None):
        """Pollt get_robot_mode bis der Mode in 'targets' liegt. -> erreichter Mode|None."""
        timeout = self.mode_timeout if timeout is None else timeout
        deadline = self.get_clock().now().nanoseconds + int(timeout * 1e9)
        last = None
        while rclpy.ok() and self.get_clock().now().nanoseconds < deadline:
            mode = self._get_robot_mode()
            if mode is not None and mode != last:
                self.get_logger().info(f"robot_mode = {_robot_mode_name(mode)}")
                last = mode
            if mode in targets:
                return mode
            self._sleep(self.mode_poll)
        return None

    def _wait_for_safety_mode(self, targets, timeout=None):
        timeout = self.mode_timeout if timeout is None else timeout
        deadline = self.get_clock().now().nanoseconds + int(timeout * 1e9)
        last = None
        while rclpy.ok() and self.get_clock().now().nanoseconds < deadline:
            mode = self._get_safety_mode()
            if mode is not None and mode != last:
                self.get_logger().info(f"safety_mode = {_safety_mode_name(mode)}")
                last = mode
            if mode in targets:
                return mode
            self._sleep(self.mode_poll)
        return None

    def _sleep(self, seconds):
        """Nicht-blockierendes Warten ueber ein Event (gibt den Thread frei)."""
        threading.Event().wait(seconds)

    def _start_external_control(self):
        """ExternalControl (re)starten. headless -> resend_robot_program, sonst load+play."""
        if self.headless_mode:
            return self._trigger(self.cli_resend_program, "resend_robot_program")

        if self.program_name:
            if not self.cli_load_program.wait_for_service(timeout_sec=self.service_timeout):
                return False, "load_program nicht verfuegbar"
            req = Load.Request()
            req.filename = self.program_name
            fut = self.cli_load_program.call_async(req)
            if not self._spin_future(fut, self.service_timeout):
                return False, "load_program Timeout"
            res = fut.result()
            self.get_logger().info(f"load_program: success={res.success} answer=\"{res.answer}\"")
            if not res.success:
                return False, f"load_program fehlgeschlagen: {res.answer}"
        return self._trigger(self.cli_play, "play")

    # ======================================================================
    # Ablaeufe (synchron, unter self._lock)
    # ======================================================================
    def prepare(self):
        """power_on -> brake_release -> ExternalControl. -> (success, message)."""
        mode = self._get_robot_mode()
        if mode is None:
            return False, "robot_mode nicht lesbar (Dashboard-Client erreichbar?)"
        self.get_logger().info(f"prepare(): Start-Mode = {_robot_mode_name(mode)}")

        # 1) Einschalten, falls noch nicht (mind.) bestromt.
        if mode in (RobotMode.POWER_OFF, RobotMode.BOOTING, RobotMode.CONFIRM_SAFETY):
            ok, msg = self._trigger(self.cli_power_on, "power_on")
            if not ok:
                return False, f"power_on fehlgeschlagen: {msg}"
            if self._wait_for_robot_mode((RobotMode.IDLE, RobotMode.POWER_ON,
                                          RobotMode.RUNNING)) is None:
                return False, "Timeout beim Warten auf POWER_ON/IDLE"
            mode = self._get_robot_mode()

        # 2) Bremsen loesen, falls noch nicht RUNNING.
        if mode != RobotMode.RUNNING:
            ok, msg = self._trigger(self.cli_brake_release, "brake_release")
            if not ok:
                return False, f"brake_release fehlgeschlagen: {msg}"
            if self._wait_for_robot_mode((RobotMode.RUNNING,)) is None:
                return False, "Timeout beim Warten auf RUNNING (brake_release)"

        # 3) ExternalControl starten, damit ROS den Arm wieder bewegen darf.
        ok, msg = self._start_external_control()
        if not ok:
            return False, f"ExternalControl-Start fehlgeschlagen: {msg}"

        return True, "Arm einsatzbereit (RUNNING, ExternalControl aktiv)"

    def recover(self):
        """Nach Safety-Violation wieder bereit machen. -> (success, message)."""
        safety = self._get_safety_mode()
        if safety is None:
            return False, "safety_mode nicht lesbar (Dashboard-Client erreichbar?)"
        self.get_logger().info(f"recover(): safety_mode = {_safety_mode_name(safety)}")

        # Eventuelle Safety-Popups wegklicken (blockieren sonst Dashboard-Befehle).
        self._trigger(self.cli_close_safety_popup, "close_safety_popup")

        if safety in (SafetyMode.NORMAL, SafetyMode.REDUCED):
            self.get_logger().info("Keine Safety-Violation -> nur prepare().")
            return self.prepare()

        if safety == SafetyMode.PROTECTIVE_STOP:
            # CB3: erst nach >=5 s loesbar.
            self.get_logger().info(
                f"Protective-Stop: warte {self.protective_stop_wait}s vor unlock ...")
            self._sleep(self.protective_stop_wait)
            ok, msg = self._trigger(self.cli_unlock_pstop, "unlock_protective_stop")
            if not ok:
                return False, f"unlock_protective_stop fehlgeschlagen: {msg}"
            if self._wait_for_safety_mode((SafetyMode.NORMAL, SafetyMode.REDUCED)) is None:
                return False, "Timeout: safety_mode bleibt nach unlock != NORMAL"
            return self.prepare()

        if safety == SafetyMode.SAFEGUARD_STOP:
            # Safeguard wird durch das physische Reset-Signal aufgehoben; danach
            # ggf. RUNNING wiederherstellen. Wir warten kurz auf NORMAL.
            self.get_logger().warn(
                "SAFEGUARD_STOP: Schutzeinrichtung physisch zuruecksetzen. "
                "Warte auf Aufhebung ...")
            if self._wait_for_safety_mode((SafetyMode.NORMAL, SafetyMode.REDUCED)) is None:
                return False, ("Safeguard-Stop besteht weiter - Schutztuer/Reset "
                               "physisch pruefen")
            return self.prepare()

        if safety in (SafetyMode.VIOLATION, SafetyMode.FAULT):
            # Safety-Controller neu starten -> Roboter geht in POWER_OFF, danach
            # voller prepare-Ablauf.
            self.get_logger().warn(
                f"{_safety_mode_name(safety)}: restart_safety (Roboter schaltet ab) ...")
            ok, msg = self._trigger(self.cli_restart_safety, "restart_safety")
            if not ok:
                return False, f"restart_safety fehlgeschlagen: {msg}"
            # Safety-Controller braucht ein paar Sekunden zum Neustart.
            self._wait_for_robot_mode((RobotMode.POWER_OFF, RobotMode.IDLE,
                                       RobotMode.POWER_ON), timeout=self.mode_timeout)
            self._trigger(self.cli_close_safety_popup, "close_safety_popup")
            if self._wait_for_safety_mode((SafetyMode.NORMAL, SafetyMode.REDUCED)) is None:
                return False, "safety_mode nach restart_safety != NORMAL"
            return self.prepare()

        if safety in (SafetyMode.ROBOT_EMERGENCY_STOP,
                      SafetyMode.SYSTEM_EMERGENCY_STOP):
            return False, ("Not-Halt aktiv - kann NICHT per Software aufgehoben werden. "
                           "E-Stop physisch entriegeln, dann recover erneut aufrufen.")

        return False, (f"safety_mode {_safety_mode_name(safety)} - keine automatische "
                       "Recovery moeglich")

    # ======================================================================
    # Service-Callbacks
    # ======================================================================
    def _run_locked(self, fn, response):
        if not self._lock.acquire(blocking=False):
            response.success = False
            response.message = "Es laeuft bereits ein prepare/recover-Vorgang"
            return response
        try:
            ok, msg = fn()
            response.success = ok
            response.message = msg
        except Exception as exc:  # defensiv: nie den Service-Thread sterben lassen
            self.get_logger().error(f"Ausnahme: {exc}")
            response.success = False
            response.message = f"Ausnahme: {exc}"
        finally:
            self._lock.release()
        return response

    def _srv_prepare(self, _request, response):
        return self._run_locked(self.prepare, response)

    def _srv_recover(self, _request, response):
        return self._run_locked(self.recover, response)

    def _srv_ensure_ready(self, _request, response):
        def fn():
            safety = self._get_safety_mode()
            if safety is None:
                return False, "safety_mode nicht lesbar"
            if safety in (SafetyMode.NORMAL, SafetyMode.REDUCED):
                return self.prepare()
            return self.recover()
        return self._run_locked(fn, response)

    def _srv_power_off(self, _request, response):
        return self._run_locked(
            lambda: self._trigger(self.cli_power_off, "power_off"), response)


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
