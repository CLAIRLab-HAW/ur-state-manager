#!/usr/bin/env python3
"""Schaltet die Arm-Controller pro Anwendungsfall um (UR5 auf a200-0553).

Idee (siehe Architektur): EIN controller_manager hostet alle Controller; die
Command-Controller (die dieselben Command-Interfaces beanspruchen und sich daher
gegenseitig ausschliessen) liegen meist INAKTIV und werden zur Laufzeit per
switch_controller aktiviert. Dieser Node bietet je einen std_srvs/Trigger-Service
pro "Modus" an; ein Aufruf aktiviert den Zielcontroller und deaktiviert die anderen
Command-Controller, die gerade aktiv sind.

Beispiel-Modi (Default, per Parameter ueberschreibbar):
  trajectory        -> arm_0_joint_trajectory_controller   (Default; MoveIt/Trajektorien)
  freedrive         -> freedrive_mode_controller           (Hand-Fuehren / Recording)
  forward_position  -> forward_position_controller         (direkte Positions-Streams)
  forward_velocity  -> forward_velocity_controller         (direkte Geschwindigkeits-Streams)
  passthrough       -> passthrough_trajectory_controller   (Trajektorien-Streaming)

Services (im Node-Namespace, z.B. /a200_0553/manipulators/ur_controller_mode_manager):
  ~/mode/<name>   (std_srvs/Trigger)  -> in diesen Modus schalten
  ~/release       (std_srvs/Trigger)  -> alle Command-Controller deaktivieren (Arm frei)
  ~/active        (std_srvs/Trigger)  -> aktuell aktive(n) Command-Controller melden

Broadcaster (joint_state_broadcaster, io_and_status_controller, ft/tcp/speed_scaling)
sind NICHT Teil der exklusiven Gruppe und bleiben unangetastet aktiv.

Voraussetzung: die genannten Controller sind im controller_manager geladen (aktiv
ODER inaktiv) - siehe arm_controllers.launch.py / config/extra_controllers.yaml.
"""

import threading

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from std_srvs.srv import Trigger
from controller_manager_msgs.srv import ListControllers, SwitchController


class ControllerModeManager(Node):
    def __init__(self):
        super().__init__("ur_controller_mode_manager")

        # controller_manager relativ -> loest im Node-Namespace auf
        cm = self.declare_parameter("controller_manager", "controller_manager").value
        cm = cm.rstrip("/")

        # Parallele Arrays: Modusname -> Controllername. Gleiche Laenge.
        self.mode_names = list(self.declare_parameter(
            "mode_names",
            ["trajectory", "freedrive", "forward_position",
             "forward_velocity", "passthrough"]).value)
        self.mode_controllers = list(self.declare_parameter(
            "mode_controllers",
            ["arm_0_joint_trajectory_controller", "freedrive_mode_controller",
             "forward_position_controller", "forward_velocity_controller",
             "passthrough_trajectory_controller"]).value)

        self.service_timeout = float(self.declare_parameter("service_timeout", 10.0).value)

        if len(self.mode_names) != len(self.mode_controllers):
            raise ValueError("mode_names und mode_controllers muessen gleich lang sein")

        # Die exklusive Gruppe = alle gemappten Command-Controller.
        self.exclusive = list(dict.fromkeys(self.mode_controllers))
        self.mode_to_controller = dict(zip(self.mode_names, self.mode_controllers))

        self.cbg = ReentrantCallbackGroup()
        self._lock = threading.Lock()

        self.cli_switch = self.create_client(
            SwitchController, f"{cm}/switch_controller", callback_group=self.cbg)
        self.cli_list = self.create_client(
            ListControllers, f"{cm}/list_controllers", callback_group=self.cbg)

        # Je ein Trigger-Service pro Modus.
        for name in self.mode_names:
            self.create_service(
                Trigger, f"~/mode/{name}",
                lambda req, resp, n=name: self._srv_set_mode(n, resp),
                callback_group=self.cbg)
        self.create_service(Trigger, "~/release", self._srv_release, callback_group=self.cbg)
        self.create_service(Trigger, "~/active", self._srv_active, callback_group=self.cbg)

        self.get_logger().info(
            f"ur_controller_mode_manager bereit. cm={cm} "
            f"modi={', '.join(self.mode_names)}")

    # ---- Low-Level ----------------------------------------------------------
    def _spin_future(self, future, timeout):
        done = threading.Event()
        future.add_done_callback(lambda _f: done.set())
        return done.wait(timeout) and future.done()

    def _active_command_controllers(self):
        """Liste der aktuell *aktiven* Controller aus der exklusiven Gruppe. None bei Fehler."""
        if not self.cli_list.wait_for_service(timeout_sec=self.service_timeout):
            return None
        fut = self.cli_list.call_async(ListControllers.Request())
        if not self._spin_future(fut, self.service_timeout):
            return None
        res = fut.result()
        active = {c.name for c in res.controller if c.state == "active"}
        loaded = {c.name for c in res.controller}
        # Merken, was ueberhaupt geladen ist (fuer aussagekraeftige Fehler).
        self._loaded = loaded
        return [c for c in self.exclusive if c in active]

    def _switch(self, activate, deactivate):
        if not self.cli_switch.wait_for_service(timeout_sec=self.service_timeout):
            return False, "switch_controller nicht verfuegbar"
        req = SwitchController.Request()
        req.activate_controllers = activate
        req.deactivate_controllers = deactivate
        req.strictness = SwitchController.Request.STRICT
        req.activate_asap = True
        fut = self.cli_switch.call_async(req)
        if not self._spin_future(fut, self.service_timeout):
            return False, "switch_controller Timeout"
        ok = fut.result().ok
        return ok, ("ok" if ok else "switch_controller meldete Fehler (geladen? Konflikt?)")

    # ---- Ablauf -------------------------------------------------------------
    def set_mode(self, mode):
        controller = self.mode_to_controller.get(mode)
        if controller is None:
            return False, f"Unbekannter Modus '{mode}'"
        self._loaded = set()
        active = self._active_command_controllers()
        if active is None:
            return False, "list_controllers fehlgeschlagen (controller_manager erreichbar?)"
        if controller not in getattr(self, "_loaded", set()):
            return False, (f"Controller '{controller}' ist nicht geladen - erst per "
                           "arm_controllers.launch.py laden")
        deactivate = [c for c in active if c != controller]
        activate = [] if controller in active else [controller]
        if not activate and not deactivate:
            return True, f"Modus '{mode}' ({controller}) bereits aktiv"
        self.get_logger().info(
            f"Modus '{mode}': activate={activate} deactivate={deactivate}")
        ok, msg = self._switch(activate, deactivate)
        if not ok:
            return False, f"Umschalten auf '{mode}' fehlgeschlagen: {msg}"
        return True, f"Modus '{mode}' aktiv ({controller})"

    def release(self):
        active = self._active_command_controllers()
        if active is None:
            return False, "list_controllers fehlgeschlagen"
        if not active:
            return True, "Kein Command-Controller aktiv"
        ok, msg = self._switch([], active)
        if not ok:
            return False, f"Deaktivieren fehlgeschlagen: {msg}"
        return True, f"Deaktiviert: {', '.join(active)}"

    # ---- Service-Callbacks --------------------------------------------------
    def _run_locked(self, fn, response):
        if not self._lock.acquire(blocking=False):
            response.success = False
            response.message = "Es laeuft bereits ein Umschaltvorgang"
            return response
        try:
            ok, msg = fn()
            response.success, response.message = ok, msg
        except Exception as exc:
            self.get_logger().error(f"Ausnahme: {exc}")
            response.success, response.message = False, f"Ausnahme: {exc}"
        finally:
            self._lock.release()
        return response

    def _srv_set_mode(self, mode, response):
        return self._run_locked(lambda: self.set_mode(mode), response)

    def _srv_release(self, _request, response):
        return self._run_locked(self.release, response)

    def _srv_active(self, _request, response):
        active = self._active_command_controllers()
        if active is None:
            response.success, response.message = False, "list_controllers fehlgeschlagen"
        else:
            response.success = True
            response.message = ", ".join(active) if active else "(keiner aktiv)"
        return response


def main():
    rclpy.init()
    node = ControllerModeManager()
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
