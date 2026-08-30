# Changelog — ur-state-manager

What changed when. The current state is described in the [README](README.md);
how it embeds into the onboard stack in
[docs/ARCHITECTURE.md](../../docs/ARCHITECTURE.md).

The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
the versioning [Semantic Versioning](https://semver.org/).

## 2026-08-30 (ruff resolves the same settings from anywhere)

- **CI pins `ruff>=0.16.5,<0.17`** -- the minor the lint scope was measured against, the same bound the
  workspace dev group carries. Unpinned, a ruff release can stabilise new rules and turn this CI red without
  a commit of ours.

## 2026-08-24 (.gitignore normalised to the workspace base)

- **`.gitignore` now uses the workspace's lean 8-line base** (`__pycache__/`, `*.py[cod]`, `*.egg-info/`, `build/`, `dist/`, `.venv/`, `.pytest_cache/`, `.DS_Store`); replaces the ~280-line auto-generated toptal.com template (Django/Flask/C/C++ patterns this package never produces). ROS extras: `install/`, `log/`, `*.pcd`, `COLCON_IGNORE`, `AMENT_IGNORE`.

## [Unreleased]

- **A GitHub workflow** (`.github/workflows/ci.yml`): pytest over `readiness.py` and `switching.py`, plus ruff for
  hard errors only. Standalone — those two modules import neither ROS nor any sibling repo, so nothing else is
  checked out and `pytest` is the only named dependency. `colcon test` is not run: the package declares no ament
  linters, so it would build a ROS workspace to execute nothing.

- **The readiness and controller-switch decisions are now ROS-free modules with a test suite** (`readiness.py`,
  `switching.py`). They previously sat inside `Node` methods that call service clients, so the cases that decide
  whether the node reaches for the real UR5 were only reachable by provoking them on the a200-0553: emergency stop,
  protective stop, safety violation, an arm powered on with ExternalControl paused, and a `switch_controller` plan
  that has to clear the incumbent claimant first. 47 tests now walk through them on any machine — the modules import
  neither `rclpy` nor `ur_dashboard_msgs`, which is installed on the robot and in none of the offboard images. They
  take mode NAMES rather than the message integers, so the `ROBOT_MODE_NAMES`/`SAFETY_MODE_NAMES` tables stay the one
  bridge to the message package instead of a second copy of its constants.

  Two behaviour changes fell out of it. `_verify_ready` now distinguishes "a fresh bring-up can heal this"
  (`retryable`) from "looking again will not change it" (`settled`), which were conflated in one flag before; the
  polling loop reads the second, the retry loop the first. And `_already_ready` and `_verify_ready` share one
  readiness predicate and one state description — they carried two copies of the `NORMAL`/`REDUCED` tuple and two
  slightly different detail formats.

- **`ament_copyright`, `ament_flake8` and `ament_pep257` are no longer declared as `test_depend`.** The package never
  had the `test/` directory that would run them, and the workspace formats with black and configures no other linter;
  `ament_flake8` would have brought a second one at 99 columns against the workspace's 120, and `ament_pep257`
  docstring conventions against its plain-reStructuredText norm.


- **`auto_recover:=false` had no effect at all -- the launch file never passed the switch on.** The node declares
  `auto_recover` (default `True`), `auto_recover_period` and `auto_recover_settle` as its own parameters, and the
  README listed all three as launch arguments, but `ur_state_manager.launch.py` neither declared them nor put them
  into the node's `parameters=`; `ros2 launch ... auto_recover:=false` was swallowed silently. Measured on
  2026-08-27 at the a200-0553, whose unit starts with exactly that argument: `ros2 param get
  .../ur_state_manager auto_recover` answered `True`, and six seconds after a bare `dashboard_client/power_on` the
  watcher took the arm from `IDLE` to `RUNNING`, released the brakes and started ExternalControl -- the very thing
  the wrapper's own comment says it switches off. The three arguments are now declared and handed over as
  `ParameterValue(..., value_type=...)`; the explicit type matters, because a `LaunchConfiguration` yields a string
  and `bool("false")` is `True`.

- **The package is now English throughout the interpreter-visible layer** -- the remaining German log lines, `Trigger`
  response messages, launch-argument descriptions and the `setup.py` description follow the comments and docstrings
  that were pulled along earlier. What a caller receives from `~/prepare`/`~/recover`/`~/ensure_ready`/`~/power_off`
  therefore changed wording (`"bereits einsatzbereit ..."` -> `"already in service (RUNNING, ExternalControl
  active)"`, `"Es laeuft bereits ein prepare/recover-Vorgang"` -> `"a prepare/recover process is already underway"`,
  and so on). No caller in the workspace matches on those strings -- checked on 2026-08-26 across every `.py`, `.md`,
  `.sh`, `.yaml` and `.js` outside this repo -- so this is prose only, no behaviour change. The `CHANGELOG` entries
  below stay German: each describes what held on its own date.

- **Die Voraussetzungsliste begruendete den `io_and_status_controller` mit dem
  Greifer** -- "on a200-0553 it is needed for the RG6 anyway". Das stammt aus
  der Tool-DO-Zeit. Seit dem URCap-Umstieg kommandiert `rg6_grip_bridge` den
  RG6 per XML-RPC, kein Tool-Ausgang ist mehr beteiligt. Gebraucht wird der
  Controller vom `robot_state_helper` -- was der naechste Satz ohnehin schon
  sagte. Am 2026-08-24 im Zuge einer Durchsicht nach Tool-DO-Resten gefunden;
  im Code selbst gab es keine: `state_manager.py` fasst den Greifer nirgends
  an, und `extra_controllers.yaml` spawnt keinen Greifer-Controller.


## [0.2.0] - 2026-08-19 (Greifer-Satz nachgezogen)

- **Der `auto_recover`-Abschnitt versprach immer noch, der Greifer komme von
  selbst mit hoch** -- "the `rg6_control` program edge pulls up tool power +
  prime automatically". Diese Programmflanke gibt es nicht mehr: der RG6 haengt
  an der OnRobot-URCap, und kein ROS-Service kann seine Tool-Versorgung setzen.
  Der Satz ist beim vorigen Aufraeumen zwar angefasst worden (das "anymore"
  fiel weg), die Tatsachenbehauptung darin blieb aber stehen.
- Die Beschreibung von `load_arm_controllers` nannte die abgeloeste
  systemd-Unit samt Datum. Warum der Launch das mitstartet, steht jetzt ohne
  Vorgeschichte da; was einmal war, steht hier.

---

**Vor der Einführung von SemVer (2026-08-19)** wurde nach Datum
geführt. Die Abschnitte darunter behalten ihre Datumsüberschrift — ihnen
nachträglich Versionsnummern zu geben, würde eine Release-Historie
erfinden, die es nicht gab.
- **SemVer eingeführt.** Version auf `0.2.0`, dieses Changelog folgt
  [Keep a Changelog](https://keepachangelog.com/de/1.1.0/), Tag `v0.2.0`.
  Ältere Abschnitte behalten ihre Datumsüberschrift — ihnen nachträglich
  Versionsnummern zu geben, würde eine Release-Historie erfinden.
- **README nach dem Workspace-Schema** (readme.so): Features · Tech Stack ·
  Installation · Usage · Running Tests · Related · Versioning · License. Die
  vorhandene Prosa ist erhalten und unter den passenden Abschnitt gewandert.
## 2026-07-29

- Hochlauf-Verifikation im Adapter: nach jedem SetMode wird selbst geprüft
  (RUNNING + Safety NORMAL/REDUCED + ExternalControl läuft) und der Hochlauf bei
  Bedarf wiederholt (`bringup_attempts`, Default 3) — inklusive CB3-Wartezeit und
  FAULT-Pfad (`restart_safety`). Ein `success=True` auf dem Wire heisst seither,
  dass der Arm wirklich bereit ist; ein absorbierter Bremsenlöse-Stop zeigt sich
  nur noch als längere Dauer (~30–70 s statt ~15 s) bzw. im `message`-Feld.
  Vorgeschichte: die Log-Forensik in
  [apps/arm-bringup](../../apps/arm-bringup/README.md).
- `arm_controllers.launch.py`: ein Wrapper fragt `list_controllers` ab und spawnt
  nur die fehlenden Controller. Vorher liess ein `systemctl restart
  clearpath-custom-ur-state-manager` gegen einen bereits bestückten
  controller_manager die Spawner erneut laden → `ros2_control_node` starb mit
  SIGSEGV in `libur_controllers.so`. Dreimal reproduziert, danach verifiziert:
  derselbe Restart ist ein No-op (CM-PID unverändert), der frische CM spawnt
  weiterhin alle sieben.
- `arm-controllers` in `ur_state_manager.launch.py` gemergt (Argument
  `load_arm_controllers`); der eigene Service `clearpath-custom-arm-controllers`
  entfiel damit (s.
  [husky-custom-setup](../husky-custom-setup/CHANGELOG.md)).
- `controller_mode_manager` läuft im selben Launch wie der `ur_state_manager`.

## 2026-07

- `ur_state_manager` delegiert an den offiziellen `robot_state_helper`
  (ur_robot_driver) statt an eine eigene State-Machine. Der Node ist seither ein
  dünner Adapter — `std_srvs/Trigger`-API (`prepare`/`recover`/`ensure_ready`/
  `power_off`) und Node-Name blieben unverändert. Plan:
  [state-manager-refactor-plan.md](../../docs/state-manager-refactor-plan.md).
