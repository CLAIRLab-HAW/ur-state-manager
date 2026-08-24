# Changelog — ur-state-manager

Was sich wann geändert hat. Der aktuelle Stand steht in der [README](README.md);
die Einbettung in den Onboard-Stack in
[docs/ARCHITECTURE.md](../../docs/ARCHITECTURE.md).

Das Format folgt [Keep a Changelog](https://keepachangelog.com/de/1.1.0/),
die Versionierung [Semantic Versioning](https://semver.org/lang/de/).

## [Unreleased]

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
