#!/usr/bin/env python3
"""Duenner Adapter auf den offiziellen 'robot_state_helper' (ur_robot_driver).

Frueher enthielt diese Datei eine eigene Mode-/Safety-Zustandsmaschine. Die ist
jetzt durch den gepflegten 'robot_state_helper' aus dem ur_robot_driver ersetzt.
Dieser Node ist nur noch ein *Adapter*: er behaelt die gewohnte
std_srvs/Trigger-API (prepare / recover / ensure_ready / power_off) und den
Node-Namen 'ur_state_manager' bei, damit bestehende Aufrufer (ur-state-manager
.service, Skripte, robot.yaml-Integration) unveraendert weiterlaufen, und
delegiert die eigentliche Arbeit an dessen ur_dashboard_msgs/action/SetMode-Action.

Was robot_state_helper alles selbst macht (und wir daher NICHT mehr nachbauen):
  * power_on -> brake_release -> RUNNING (schrittweise Mode-Transition),
  * unlock_protective_stop bei PROTECTIVE_STOP,
  * restart_safety bei VIOLATION / FAULT,
  * ExternalControl (re)starten: headless_mode -> resend_robot_program, sonst play,
  * E-Stop wird nur gemeldet (nicht per Software loesbar).

Einzige Zutat, die robot_state_helper NICHT kennt: die CB3-Pflicht, nach einem
Protective-Stop >=5 s zu warten, bevor unlock_protective_stop akzeptiert wird.
robot_state_helper unlockt sofort -> auf dem CB3 kann das fehlschlagen. Deshalb
liest 'recover'/'ensure_ready' vorher den safety_mode (Dashboard-Client) und
wartet ggf. kurz, BEVOR das SetMode-Goal (das intern sofort unlockt) rausgeht.

Mapping der Trigger-Services auf SetMode-Goals:
  ~/prepare       [idempotent] SetMode{RUNNING, stop_program=false, play_program=true}
                  Vorcheck: ist der Arm schon RUNNING + ExternalControl aktiv +
                  Safety NORMAL/REDUCED, gibt es nichts zu tun -> success=True OHNE
                  robot_state_helper (wichtig fuers wiederholte Starten der Demo).
  ~/recover       [pstop-wait] SetMode{RUNNING, stop_program=true, play_program=true}
  ~/ensure_ready  wie recover (SetMode macht ohnehin "whatever it takes")
  ~/power_off     SetMode{POWER_OFF, stop_program=true,  play_program=false}

Alle Namen sind Parameter (Defaults passen zu a200-0553).
"""

import threading

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from ur_dashboard_msgs.action import SetMode
from ur_dashboard_msgs.msg import RobotMode, SafetyMode
from ur_dashboard_msgs.srv import GetRobotMode, GetSafetyMode


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
        # Action des robot_state_helper. Er laeuft (siehe Launch) als Node
        # 'ur_robot_state_helper' im manipulators-Namespace.
        self.set_mode_action = self.declare_parameter(
            "set_mode_action",
            "/a200_0553/manipulators/ur_robot_state_helper/set_mode").value
        # Nur fuer die CB3-Wartezeit vor dem (intern sofortigen) unlock noetig;
        # zusaetzlich fuer den idempotenten prepare-Vorcheck (get_robot_mode).
        dashboard_ns = self.declare_parameter(
            "dashboard_ns",
            "/a200_0553/manipulators/dashboard_client").value.rstrip("/")
        # io_and_status_controller: liefert robot_program_running (ExternalControl aktiv?)
        # fuer den idempotenten prepare-Vorcheck.
        io_status_ns = self.declare_parameter(
            "io_status_ns",
            "/a200_0553/manipulators/io_and_status_controller").value.rstrip("/")

        self.service_timeout = float(self.declare_parameter("service_timeout", 10.0).value)
        # Wie lange ein Mode-Uebergang (z.B. POWER_OFF -> RUNNING) dauern darf.
        self.action_timeout = float(self.declare_parameter("action_timeout", 120.0).value)
        # CB3 verweigert das Loesen eines Protective-Stops < 5 s nach dem Ausloesen.
        self.protective_stop_wait = float(self.declare_parameter("protective_stop_wait", 6.0).value)

        # Clients + Server in einer ReentrantCallbackGroup, damit wir synchron aus
        # einem Service-Callback heraus die Action abwarten koennen (Antwort wird
        # von einem anderen Thread des MultiThreadedExecutor verarbeitet).
        self.cbg = ReentrantCallbackGroup()

        self.cli_set_mode = ActionClient(
            self, SetMode, self.set_mode_action, callback_group=self.cbg)
        self.cli_get_safety_mode = self.create_client(
            GetSafetyMode, f"{dashboard_ns}/get_safety_mode", callback_group=self.cbg)
        self.cli_get_robot_mode = self.create_client(
            GetRobotMode, f"{dashboard_ns}/get_robot_mode", callback_group=self.cbg)

        # ExternalControl-Status (True = ROS-Programm laeuft) fuer den idempotenten
        # prepare-Vorcheck. Wird periodisch vom io_and_status_controller gepublisht;
        # None = noch nichts empfangen -> Vorcheck greift nicht (sicherer Default).
        self._program_running = None
        self.create_subscription(
            Bool, f"{io_status_ns}/robot_program_running",
            self._on_program_running, 1, callback_group=self.cbg)

        # ---- Eigene Services (unveraendert zur alten API) -----------------
        self._lock = threading.Lock()  # nie zwei Ablaeufe gleichzeitig
        self.create_service(Trigger, "~/prepare", self._srv_prepare, callback_group=self.cbg)
        self.create_service(Trigger, "~/recover", self._srv_recover, callback_group=self.cbg)
        self.create_service(Trigger, "~/ensure_ready", self._srv_ensure_ready, callback_group=self.cbg)
        self.create_service(Trigger, "~/power_off", self._srv_power_off, callback_group=self.cbg)

        # ---- Auto-Recovery-Watcher (spaetes Einschalten des Arms) ----------
        # Wird der UR erst NACH dem Boot bestromt, laeuft ExternalControl nicht an
        # (Teach-Panel "Paused", Arm ohne Feedback, Greifer stromlos). Dieser Watcher
        # erkennt "bestromt, aber ExternalControl aus" und ruft selbsttaetig recover
        # -> RUNNING + frisches ExternalControl. recover nutzt stop_program=True (sauberer
        # Neustart -> Treiber sync't Command=Ist -> KEIN Positionssprung/Protective-Stop,
        # anders als ein blosses prepare/play, das den Paused-Stand mit stale Command
        # fortsetzt). Danach zieht die rg6_control-Programm-Flanke Tool-Power + Prime nach.
        # auto_recover=false schaltet den Automatismus ab.
        self.auto_recover = bool(self.declare_parameter("auto_recover", True).value)
        self.auto_recover_period = float(
            self.declare_parameter("auto_recover_period", 5.0).value)
        # so viele aufeinanderfolgende "muss recovern"-Beobachtungen vor dem Handeln
        # (entprellt Boot-/prepare-Uebergaenge, in denen der Zustand kurz passt).
        self.auto_recover_settle = int(self.declare_parameter("auto_recover_settle", 2).value)
        self._needs_recover_count = 0
        if self.auto_recover:
            self.create_timer(
                self.auto_recover_period, self._auto_recover_tick, callback_group=self.cbg)

        self.get_logger().info(
            f"ur_state_manager (Adapter) bereit. set_mode_action={self.set_mode_action} "
            f"dashboard_ns={dashboard_ns} auto_recover={self.auto_recover}")

    # ======================================================================
    # Low-Level-Helfer
    # ======================================================================
    def _spin_future(self, future, timeout):
        """Auf ein *_async-Future warten, ohne den Executor-Thread zu blockieren."""
        done = threading.Event()
        future.add_done_callback(lambda _f: done.set())
        return done.wait(timeout) and future.done()

    def _sleep(self, seconds):
        """Nicht-blockierendes Warten (gibt den Thread frei)."""
        threading.Event().wait(seconds)

    def _on_feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f"SetMode-Feedback: robot_mode={_robot_mode_name(fb.current_robot_mode)} "
            f"safety_mode={_safety_mode_name(fb.current_safety_mode)}")

    def _on_program_running(self, msg: Bool):
        self._program_running = bool(msg.data)

    def _get_safety_mode(self):
        """safety_mode ueber den Dashboard-Client lesen. -> mode | None."""
        if not self.cli_get_safety_mode.wait_for_service(timeout_sec=self.service_timeout):
            return None
        fut = self.cli_get_safety_mode.call_async(GetSafetyMode.Request())
        if not self._spin_future(fut, self.service_timeout):
            return None
        return fut.result().safety_mode.mode

    def _get_robot_mode(self):
        """robot_mode ueber den Dashboard-Client lesen. -> mode | None."""
        if not self.cli_get_robot_mode.wait_for_service(timeout_sec=self.service_timeout):
            return None
        fut = self.cli_get_robot_mode.call_async(GetRobotMode.Request())
        if not self._spin_future(fut, self.service_timeout):
            return None
        return fut.result().robot_mode.mode

    def _already_ready(self):
        """Idempotenz-Check fuer prepare: ist der Arm bereits einsatzbereit
        (RUNNING + Safety NORMAL/REDUCED + ExternalControl aktiv), sodass KEIN
        Mode-Wechsel und damit kein robot_state_helper noetig ist? -> bool."""
        robot_mode = self._get_robot_mode()
        safety = self._get_safety_mode()
        prog = self._program_running
        if (robot_mode == RobotMode.RUNNING
                and safety in (SafetyMode.NORMAL, SafetyMode.REDUCED)
                and prog is True):
            self.get_logger().info(
                "prepare: Arm bereits RUNNING + ExternalControl aktiv "
                "-> kein Mode-Wechsel noetig (robot_state_helper nicht gebraucht).")
            return True
        self.get_logger().info(
            "prepare: nicht direkt bereit (robot_mode="
            f"{_robot_mode_name(robot_mode) if robot_mode is not None else 'unbekannt'}, "
            f"safety={_safety_mode_name(safety) if safety is not None else 'unbekannt'}, "
            f"program_running={prog}) -> delegiere an robot_state_helper.")
        return False

    def _wait_if_protective_stop(self):
        """CB3: nach Protective-Stop >=5 s warten, bevor robot_state_helper unlockt."""
        safety = self._get_safety_mode()
        if safety == SafetyMode.PROTECTIVE_STOP:
            self.get_logger().info(
                f"Protective-Stop erkannt -> warte {self.protective_stop_wait}s "
                "(CB3-Pflicht) vor dem unlock ...")
            self._sleep(self.protective_stop_wait)
        elif safety is None:
            self.get_logger().warn(
                "safety_mode nicht lesbar (Dashboard-Client da?) - fahre ohne "
                "CB3-Wartezeit fort; ggf. recover erneut aufrufen.")

    def _set_mode(self, target, stop_program, play_program):
        """SetMode-Goal senden und synchron auf das Ergebnis warten. -> (ok, msg)."""
        if not self.cli_set_mode.wait_for_server(timeout_sec=self.service_timeout):
            return False, ("robot_state_helper/set_mode-Action nicht verfuegbar - "
                           "laeuft der ur_robot_state_helper-Node?")

        goal = SetMode.Goal()
        goal.target_robot_mode = target
        goal.stop_program = stop_program
        goal.play_program = play_program
        self.get_logger().info(
            f"SetMode -> target={_robot_mode_name(target)} "
            f"stop_program={stop_program} play_program={play_program}")

        send_fut = self.cli_set_mode.send_goal_async(goal, feedback_callback=self._on_feedback)
        if not self._spin_future(send_fut, self.service_timeout):
            return False, "SetMode: Timeout beim Senden des Goals"
        handle = send_fut.result()
        if not handle.accepted:
            return False, "SetMode-Goal abgelehnt (laeuft schon ein Vorgang im robot_state_helper?)"

        res_fut = handle.get_result_async()
        if not self._spin_future(res_fut, self.action_timeout):
            return False, f"SetMode: Timeout ({self.action_timeout}s) beim Warten auf das Ergebnis"
        result = res_fut.result().result
        return result.success, result.message

    # ======================================================================
    # Ablaeufe (delegieren an robot_state_helper)
    # ======================================================================
    def prepare(self):
        """Arm einsatzbereit: RUNNING + ExternalControl (aus POWER_OFF hochfahren).

        Idempotent: ist der Arm schon einsatzbereit, wird sofort success=True
        gemeldet, OHNE den robot_state_helper zu benoetigen. So laeuft die Demo
        auch beim wiederholten Start (Arm bereits RUNNING) durch, selbst wenn der
        robot_state_helper gerade nicht erreichbar ist.
        """
        if self._already_ready():
            return True, "bereits einsatzbereit (RUNNING, ExternalControl aktiv)"
        return self._set_mode(RobotMode.RUNNING, stop_program=False, play_program=True)

    def recover(self):
        """Nach Safety-Violation wieder bereit: Programm stoppen, RUNNING, neu starten.

        robot_state_helper behandelt PROTECTIVE_STOP / VIOLATION / FAULT / E-Stop
        selbst; wir warten davor nur die CB3-Pflichtzeit ab. stop_program=true
        entspricht der UR-Empfehlung, nach einem Stop das Programm NEU zu starten
        (statt es einfach fortzusetzen).
        """
        self._wait_if_protective_stop()
        return self._set_mode(RobotMode.RUNNING, stop_program=True, play_program=True)

    def power_off(self):
        """Arm sicher abschalten."""
        return self._set_mode(RobotMode.POWER_OFF, stop_program=True, play_program=False)

    # ======================================================================
    # Auto-Recovery: bringt den Arm nach spaetem Einschalten ohne Handgriff hoch
    # ======================================================================
    def _needs_recover(self):
        """True, wenn der Arm bestromt ist, ExternalControl aber NICHT laeuft.

        Genau der Zustand nach spaetem Einschalten / 'Paused': robot_mode in
        {POWER_ON, IDLE, RUNNING}, aber robot_program_running=False. POWER_OFF /
        DISCONNECTED / BOOTING (Arm bewusst aus bzw. faehrt noch hoch) und
        BACKDRIVE (Freedrive) werden NICHT angefasst. Unbekannter Programmstatus
        (None, noch nichts empfangen) -> nicht handeln (sicherer Default)."""
        if self._program_running is not False:
            return False  # laeuft schon, oder Status noch unbekannt
        mode = self._get_robot_mode()
        return mode in (RobotMode.POWER_ON, RobotMode.IDLE, RobotMode.RUNNING)

    def _auto_recover_tick(self):
        # Laeuft schon ein prepare/recover (manuell ODER auto)? -> nicht reinfunken.
        if self._lock.locked():
            self._needs_recover_count = 0
            return
        if not self._needs_recover():
            self._needs_recover_count = 0
            return
        self._needs_recover_count += 1
        if self._needs_recover_count < max(1, self.auto_recover_settle):
            return  # entprellen: erst nach mehreren konsistenten Beobachtungen handeln
        self._needs_recover_count = 0
        self.get_logger().warn(
            "Auto-Recovery: Arm bestromt, aber ExternalControl laeuft nicht "
            "(spaetes Einschalten / Paused) -> fuehre recover aus ...")
        resp = Trigger.Response()
        self._run_locked(self.recover, resp)
        self.get_logger().info(
            f"Auto-Recovery: recover -> success={resp.success} ({resp.message})")

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
        # SetMode macht ohnehin "was noetig ist" -> identisch zu recover (inkl. CB3-Wait).
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
