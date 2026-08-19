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

Vier Zutaten gegen die Restart-/Erstbestromungs-Fallen (2026-07-29, a200-0553):

* VERIFIKATION + RETRY: Beim ERSTEN Bremsenloesen, nachdem der Arm eine Weile
  stromlos war, wirft die CB3 haeufig einen Protective Stop oder FAULT aus der
  eigenen Anlauf-Prozedur (C153A3/C204A1 bzw. C39/C193 "beim Loesen sackt ein
  Gelenk ueber die Toleranz"), BEVOR ROS irgendetwas streamt; der zweite Anlauf
  laeuft danach zuverlaessig durch. robot_state_helper merkt davon nichts: sein
  SetMode-Goal meldet success, sobald RUNNING erreicht und play/resend abgesetzt
  ist - der P-Stop faellt in die Luecke dazwischen. Deshalb prueft dieser Adapter
  nach jedem SetMode selbst (RUNNING + Safety NORMAL/REDUCED + ExternalControl
  laeuft) und wiederholt den Hochlauf bei Bedarf (bringup_attempts, Default 3).
* HELPER-PRIMING: robot_state_helper abonniert robot_mode/safety_mode
  BEST_EFFORT+VOLATILE; der GPIOController publiziert TRANSIENT_LOCAL und nur
  bei AENDERUNG. Nach einem Restart nur dieses Services (ohne Treiber-Restart)
  bleibt der Helper daher blind ("Robot mode is unknown") und lehnt jedes Goal
  ab - es publiziert ja niemand neu. Vor jedem Goal publiziert dieser Adapter
  den per Dashboard gelesenen Ist-Stand EINMAL (VOLATILE, latcht nichts) auf
  dieselben Topics und macht den Helper damit deterministisch sehend.
* CONTROLLER-RELEASE vor dem Mode-Zyklus: Nach einem manipulators-Restart ist
  der arm_0_joint_trajectory_controller aktiv, sein Halteziel stammt aber von
  VOR dem Bestromen (Bremsen zu). Startet ExternalControl mit diesem stale
  Haltewert, streamt der Treiber sofort dorthin -> Positionssprung. Release vor
  dem Hochlauf + frisches Aktivieren danach (_ensure_trajectory_mode) schliesst
  die Luecke.
* LATCHED-QoS + DASHBOARD-FALLBACK: robot_program_running kommt
  TRANSIENT_LOCAL und nur bei Aenderung -> Subscription hier ebenfalls
  TRANSIENT_LOCAL. Unter rmw_zenoh kommt der latched Wert bei Late-Joinern
  trotzdem NICHT zuverlaessig an (empirisch; nur Live-Aenderungen) -> solange
  das Topic nichts geliefert hat, springt der Dashboard-Server ein
  ('program_running'; im headless-Betrieb ist das ExternalControl-Skript das
  laufende Programm). Ohne beides waeren Vorcheck, Verifikation und
  Auto-Recovery nach jedem Adapter-Restart blind (_program_running=None).

Mapping der Trigger-Services auf SetMode-Goals:
  ~/prepare       [idempotent] SetMode{RUNNING, stop_program=false, play_program=true}
                  Vorcheck: ist der Arm schon RUNNING + ExternalControl aktiv +
                  Safety NORMAL/REDUCED, gibt es nichts zu tun -> success=True OHNE
                  robot_state_helper (wichtig fuers wiederholte Starten der Demo).
                  Retries laufen als recover (stop_program=true, sauberer Neustart).
  ~/recover       [pstop-wait] SetMode{RUNNING, stop_program=true, play_program=true}
  ~/ensure_ready  wie recover (SetMode macht ohnehin "whatever it takes")
  ~/power_off     SetMode{POWER_OFF, stop_program=true,  play_program=false}

Alle Namen sind Parameter (Defaults passen zu a200-0553).
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
        # controller_mode_manager: nach einem erfolgreichen prepare/recover wird
        # zusaetzlich der Trajectory-Modus aktiviert. Hintergrund: ein power_off
        # stoppt das ExternalControl-Programm -> der Treiber meldet seine
        # Command-Interfaces als nicht verfuegbar -> ros2_control MUSS jeden
        # Controller deaktivieren, der sie beansprucht (im Log: "Successful
        # 'deactivate' of hardware 'arm_0'" + "Deactivating controllers:
        # [arm_0_joint_trajectory_controller]"). Beim Hochfahren aktiviert
        # ros2_control ihn NICHT von selbst wieder. Ohne diesen Schritt bleibt der
        # Arm also bestromt und verbunden, aber jede MoveIt-Ausfuehrung scheitert -
        # ohne dass die Fehlermeldung auf den inaktiven Controller zeigt.
        mode_manager_ns = self.declare_parameter(
            "mode_manager_ns",
            "/a200_0553/manipulators/ur_controller_mode_manager").value.rstrip("/")
        self.trajectory_mode = self.declare_parameter(
            "trajectory_mode", "trajectory").value
        self.ensure_trajectory_mode = bool(self.declare_parameter(
            "ensure_trajectory_mode", True).value)

        self.service_timeout = float(self.declare_parameter("service_timeout", 10.0).value)
        # Wie lange ein Mode-Uebergang (z.B. POWER_OFF -> RUNNING) dauern darf.
        self.action_timeout = float(self.declare_parameter("action_timeout", 120.0).value)
        # CB3 verweigert das Loesen eines Protective-Stops < 5 s nach dem Ausloesen.
        self.protective_stop_wait = float(self.declare_parameter("protective_stop_wait", 6.0).value)
        # Nach einem "erfolgreichen" SetMode: so lange darf es dauern, bis der Arm
        # WIRKLICH bereit ist (RUNNING + Safety NORMAL/REDUCED + ExternalControl
        # laeuft). Ein Protective Stop/FAULT bricht die Wartezeit sofort ab.
        self.verify_ready_timeout = float(
            self.declare_parameter("verify_ready_timeout", 20.0).value)
        # Hochlauf-Anlaeufe insgesamt. Der CB3-Bremsenloese-P-Stop (Modul-Docstring)
        # heilt empirisch immer im zweiten Anlauf; 3 laesst Luft fuer einen FAULT
        # (restart_safety) dazwischen.
        self.bringup_attempts = int(self.declare_parameter("bringup_attempts", 3).value)
        # Command-Controller (JTC & Co.) vor jedem Mode-Zyklus deaktivieren.
        self.release_before_power_cycle = bool(self.declare_parameter(
            "release_before_power_cycle", True).value)

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
        self.cli_program_running = self.create_client(
            IsProgramRunning, f"{dashboard_ns}/program_running", callback_group=self.cbg)
        self.cli_trajectory_mode = self.create_client(
            Trigger, f"{mode_manager_ns}/mode/{self.trajectory_mode}",
            callback_group=self.cbg)
        self.cli_release = self.create_client(
            Trigger, f"{mode_manager_ns}/release", callback_group=self.cbg)

        # Priming-Publisher (Modul-Docstring): einmalige Ist-Stand-Publikation auf
        # den GPIOController-Topics, damit der BEST_EFFORT/VOLATILE-Subscriber im
        # robot_state_helper nach (Teil-)Restarts nicht blind bleibt. Bewusst
        # VOLATILE: es darf NICHTS latchen, was spaeter als Stale-Sample bei
        # Late-Joinern landet - der GPIOController bleibt Owner der Topics.
        self.pub_robot_mode = self.create_publisher(
            RobotMode, f"{io_status_ns}/robot_mode", 1)
        self.pub_safety_mode = self.create_publisher(
            SafetyMode, f"{io_status_ns}/safety_mode", 1)

        # ExternalControl-Status (True = ROS-Programm laeuft) fuer den idempotenten
        # prepare-Vorcheck und die Hochlauf-Verifikation. Der GPIOController
        # publisht TRANSIENT_LOCAL und NUR bei Aenderung -> Subscription ebenfalls
        # TRANSIENT_LOCAL, sonst bleibt der Wert nach einem Adapter-Restart None,
        # bis sich das Programm das naechste Mal aendert (Vorcheck+Watcher blind).
        self._program_running = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            Bool, f"{io_status_ns}/robot_program_running",
            self._on_program_running, latched, callback_group=self.cbg)

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
        # fortsetzt). Den Greifer betrifft das seit 2026-08-19 nicht mehr: er haengt
        # an der OnRobot-URCap, nicht am Tool-Anschluss, und braucht weder
        # Tool-Power aus ROS noch ein Priming auf der Programm-Flanke.
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

    def _effective_program_running(self):
        """ExternalControl-Status: Topic-Wert, sonst Dashboard-Fallback. -> bool | None.

        Der latched robot_program_running-Wert kommt unter rmw_zenoh bei einem
        Late-Joiner NICHT zuverlaessig an (nur Live-Aenderungen; empirisch am
        a200-0553). Solange das Topic also noch nichts geliefert hat, fragt
        dieser Fallback den Dashboard-Server ('program_running') - im
        headless-Betrieb laeuft dort das ExternalControl-Skript als Programm."""
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
        """Idempotenz-Check fuer prepare: ist der Arm bereits einsatzbereit
        (RUNNING + Safety NORMAL/REDUCED + ExternalControl aktiv), sodass KEIN
        Mode-Wechsel und damit kein robot_state_helper noetig ist? -> bool."""
        robot_mode = self._get_robot_mode()
        safety = self._get_safety_mode()
        prog = self._effective_program_running()
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

    def _prime_state_helper(self):
        """robot_state_helper vor jedem Goal 'sehend' machen (Modul-Docstring).

        Ist-Stand per Dashboard lesen und EINMAL auf die robot_mode/safety_mode-
        Topics publizieren. Ohne das lehnt der Helper nach einem Restart dieses
        Services (ohne Treiber-Restart) jedes Goal ab, weil er die latched Werte
        des GPIOController verpasst hat und niemand neu publiziert."""
        robot_mode = self._get_robot_mode()
        safety = self._get_safety_mode()
        if robot_mode is None and safety is None:
            self.get_logger().warn(
                "Priming uebersprungen: Dashboard liefert weder robot_mode noch "
                "safety_mode - Goal kann am 'unknown mode'-Check des Helpers "
                "scheitern.")
            return
        if robot_mode is not None:
            self.pub_robot_mode.publish(RobotMode(mode=int(robot_mode)))
        if safety is not None:
            self.pub_safety_mode.publish(SafetyMode(mode=int(safety)))
        # dem Helper einen Moment lassen, die Samples vor dem Goal zu verarbeiten
        self._sleep(0.3)

    def _set_mode(self, target, stop_program, play_program):
        """SetMode-Goal senden und synchron auf das Ergebnis warten. -> (ok, msg)."""
        if not self.cli_set_mode.wait_for_server(timeout_sec=self.service_timeout):
            return False, ("robot_state_helper/set_mode-Action nicht verfuegbar - "
                           "laeuft der ur_robot_state_helper-Node?")
        self._prime_state_helper()

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
            # Upstream (jazzy) lehnt NUR ab, wenn robot_mode/safety_mode noch
            # UNKNOWN/UNDEFINED sind (Helper hat vom frisch gestarteten Treiber
            # noch keine Statusdaten; unter rmw_zenoh dauert die Discovery ein
            # paar Sekunden). Einen Busy-Check gibt es upstream nicht -
            # konkurrierende Goals werden angenommen.
            return False, ("SetMode-Goal abgelehnt - robot_state_helper vermutlich "
                           "noch nicht ready (robot_mode/safety_mode noch nicht "
                           "empfangen, z.B. direkt nach Stack-Restart); naechster "
                           "Versuch heilt das i.d.R.")

        res_fut = handle.get_result_async()
        if not self._spin_future(res_fut, self.action_timeout):
            return False, f"SetMode: Timeout ({self.action_timeout}s) beim Warten auf das Ergebnis"
        result = res_fut.result().result
        return result.success, result.message

    # ======================================================================
    # Ablaeufe (delegieren an robot_state_helper)
    # ======================================================================
    def _ensure_trajectory_mode(self):
        """Trajectory-Controller aktivieren (best effort, nie fatal).

        Wird nach erfolgreichem prepare/recover gerufen. Idempotent: der
        controller_mode_manager schaltet nur, wenn noetig. Schlaegt es fehl (Mode-
        Manager nicht da, Timeout), wird nur gewarnt - der Arm ist dann bestromt
        und verbunden, nur der Controller fehlt; das ist besser als ein prepare,
        das deswegen als Fehler gilt."""
        if not self.ensure_trajectory_mode:
            return
        if not self.cli_trajectory_mode.wait_for_service(timeout_sec=self.service_timeout):
            self.get_logger().warn(
                f"Trajectory-Modus: {self.cli_trajectory_mode.srv_name} nicht "
                "erreichbar (laeuft der controller_mode_manager?) - der Arm ist "
                "bereit, aber MoveIt-Ausfuehrung schlaegt fehl, bis der "
                "arm_0_joint_trajectory_controller aktiv ist.")
            return
        fut = self.cli_trajectory_mode.call_async(Trigger.Request())
        if not self._spin_future(fut, self.service_timeout):
            self.get_logger().warn("Trajectory-Modus: Timeout beim Umschalten.")
            return
        res = fut.result()
        if res.success:
            self.get_logger().info(f"Trajectory-Modus aktiv ({res.message}).")
        else:
            self.get_logger().warn(f"Trajectory-Modus nicht gesetzt: {res.message}")

    def _release_command_controllers(self):
        """Command-Controller vor einem Mode-Zyklus deaktivieren (best effort).

        Nach einem manipulators-Restart ist der arm_0_joint_trajectory_controller
        aktiv, sein Halteziel stammt aber von VOR dem Bestromen (Bremsen zu).
        Startet ExternalControl mit diesem stale Haltewert, streamt der Treiber
        sofort dorthin -> Positionssprung/Schleppfehler. Release hier + frisches
        Aktivieren in _ensure_trajectory_mode NACH dem Hochlauf schliesst die
        Luecke. Nie fatal: ohne Mode-Manager laeuft der Hochlauf wie bisher."""
        if not self.release_before_power_cycle:
            return
        if not self.cli_release.wait_for_service(timeout_sec=self.service_timeout):
            self.get_logger().warn(
                f"Controller-Release: {self.cli_release.srv_name} nicht erreichbar "
                "(laeuft der controller_mode_manager?) - fahre ohne Release fort.")
            return
        fut = self.cli_release.call_async(Trigger.Request())
        if not self._spin_future(fut, self.service_timeout):
            self.get_logger().warn("Controller-Release: Timeout - fahre fort.")
            return
        res = fut.result()
        log = self.get_logger().info if res.success else self.get_logger().warn
        log(f"Controller-Release vor dem Mode-Zyklus: {res.message}")

    _GOOD_SAFETY = (SafetyMode.NORMAL, SafetyMode.REDUCED)
    _TERMINAL_SAFETY = (SafetyMode.SYSTEM_EMERGENCY_STOP,
                        SafetyMode.ROBOT_EMERGENCY_STOP)

    def _verify_ready(self, timeout):
        """Nach einem 'erfolgreichen' SetMode pruefen, ob der Arm WIRKLICH bereit ist.

        robot_state_helper meldet success, sobald RUNNING erreicht und play/resend
        abgesetzt ist - ein Protective Stop, der WAEHREND des Hochlaufs faellt
        (CB3-Bremsenloesen, Modul-Docstring), rutscht durch diese Luecke. Hier:
        RUNNING + Safety NORMAL/REDUCED + ExternalControl laeuft, gepollt bis
        ``timeout``; P-Stop/FAULT/VIOLATION bricht sofort ab (Retry heilt),
        E-Stop bricht endgueltig ab. -> (ok, detail, retryable)."""
        deadline = time.monotonic() + max(0.0, timeout)
        while True:
            robot_mode = self._get_robot_mode()
            safety = self._get_safety_mode()
            prog = self._effective_program_running()
            if (robot_mode == RobotMode.RUNNING and safety in self._GOOD_SAFETY
                    and prog is True):
                return True, "", True
            detail = (
                "robot_mode="
                f"{_robot_mode_name(robot_mode) if robot_mode is not None else 'unbekannt'} "
                f"safety={_safety_mode_name(safety) if safety is not None else 'unbekannt'} "
                f"program_running={prog}")
            if safety in self._TERMINAL_SAFETY:
                return False, f"{detail} (E-Stop: nur manuell loesbar)", False
            if safety in (SafetyMode.PROTECTIVE_STOP, SafetyMode.VIOLATION,
                          SafetyMode.FAULT):
                return False, detail, True
            if time.monotonic() >= deadline:
                return False, detail, True
            self._sleep(0.5)

    def _bringup(self, stop_program_first):
        """Hochlauf mit Verifikation + Retry (Kern von prepare/recover).

        Ein Anlauf: [CB3-P-Stop-Wartezeit] -> Controller-Release -> SetMode(RUNNING)
        -> Verifikation. Der CB3-Bremsenloese-P-Stop des ersten Anlaufs (Modul-
        Docstring) wird so INNERHALB eines Trigger-Aufrufs geheilt, statt dem
        Aufrufer ein falsches success (oder einen halbtoten Arm) zu hinterlassen.
        Ab dem zweiten Anlauf immer stop_program=True (sauberer Programm-Neustart,
        UR-Empfehlung nach jedem Stop)."""
        attempts = max(1, self.bringup_attempts)
        msg = ""
        for attempt in range(1, attempts + 1):
            if attempt > 1:
                self.get_logger().warn(
                    f"Hochlauf-Anlauf {attempt}/{attempts} (zuvor: {msg})")
            self._wait_if_protective_stop()
            self._release_command_controllers()
            ok, msg = self._set_mode(
                RobotMode.RUNNING,
                stop_program=stop_program_first or attempt > 1,
                play_program=True)
            if not ok:
                continue
            ok, detail, retryable = self._verify_ready(self.verify_ready_timeout)
            if ok:
                self._ensure_trajectory_mode()
                if attempt > 1:
                    return True, f"bereit (Anlauf {attempt}/{attempts})"
                return True, (msg or "bereit")
            msg = f"Hochlauf nicht verifiziert: {detail}"
            if not retryable:
                break
        return False, f"Arm nach {attempts} Anlaeufen nicht bereit - letzter Stand: {msg}"

    def prepare(self):
        """Arm einsatzbereit: RUNNING + ExternalControl + Trajectory-Controller.

        Idempotent: ist der Arm schon einsatzbereit, wird sofort success=True
        gemeldet, OHNE den robot_state_helper zu benoetigen. So laeuft die Demo
        auch beim wiederholten Start (Arm bereits RUNNING) durch, selbst wenn der
        robot_state_helper gerade nicht erreichbar ist.

        Der Trajectory-Modus wird in BEIDEN Faellen sichergestellt - auch im
        Idempotenz-Zweig: nach einem power_off ist der Arm zwar schnell wieder
        RUNNING, der Controller aber deaktiviert (s. Kommentar bei mode_manager_ns).
        """
        if self._already_ready():
            self._ensure_trajectory_mode()
            return True, "bereits einsatzbereit (RUNNING, ExternalControl aktiv)"
        return self._bringup(stop_program_first=False)

    def recover(self):
        """Nach Safety-Violation wieder bereit: Programm stoppen, RUNNING, neu starten.

        robot_state_helper behandelt PROTECTIVE_STOP / VIOLATION / FAULT / E-Stop
        selbst; wir warten davor nur die CB3-Pflichtzeit ab. stop_program=true
        entspricht der UR-Empfehlung, nach einem Stop das Programm NEU zu starten
        (statt es einfach fortzusetzen).
        """
        return self._bringup(stop_program_first=True)

    def power_off(self):
        """Arm sicher abschalten. Controller vorher freigeben: beim Programm-Stop
        meldet der Treiber seine Command-Interfaces ohnehin als nicht verfuegbar,
        das Release macht daraus einen geordneten Schritt statt einer Zwangs-
        Deaktivierung (und haelt kein stale Halteziel fuer den naechsten Start)."""
        self._release_command_controllers()
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
        (None, auch nach Dashboard-Fallback) -> nicht handeln (sicherer Default)."""
        if self._effective_program_running() is not False:
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
