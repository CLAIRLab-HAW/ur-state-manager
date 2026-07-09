# ur_state_manager

Verwaltet den Zustand eines **bereits verbundenen** UR5 (CB3) auf der `a200-0553`:
Arm einsatzbereit machen und nach einer Safety-Violation wieder herstellen.

> **Umbau (2026-07):** Die frühere eigene Mode-/Safety-Zustandsmaschine ist durch den
> offiziellen **`robot_state_helper`** aus dem `ur_robot_driver` ersetzt. Dieser Node ist
> jetzt nur noch ein **dünner Adapter**: er behält die gewohnte `std_srvs/Trigger`-API und
> den Node-Namen `ur_state_manager` bei (nichts Downstream bricht) und delegiert an die
> `ur_dashboard_msgs/action/SetMode`-Action des `robot_state_helper`.
>
> Der `robot_state_helper` erledigt selbst: `power_on` → `brake_release` → `RUNNING`,
> `unlock_protective_stop`, `restart_safety` bei `VIOLATION`/`FAULT`, sowie
> ExternalControl (headless: `resend_robot_program`, sonst `play`). Das Launch dieses
> Pakets startet ihn mit.

Er deckt zwei Aufgaben ab:

1. **Einsatzbereit machen** – `power_on` → `brake_release` → ExternalControl starten.
2. **Recovery nach Safety-Violation** – Arm ist nach Kollision / Protective-Stop in den
   Lock-/Stop-State gefallen und soll wieder bereit werden.

> Der Node **bewegt den Arm nicht** und sendet keine Trajektorien. Er stellt nur sicher,
> dass der Arm bestromt, Bremsen gelöst und ROS-kontrollierbar (ExternalControl aktiv) ist.

## Angebotene Services (alle `std_srvs/Trigger`)

Jeder Service übersetzt in ein `SetMode`-Goal an den `robot_state_helper`:

| Service | Delegiert an `SetMode` | Wirkung |
|---|---|---|
| `~/prepare` | `{RUNNING, play_program}` | Hochfahren bis `RUNNING` + ExternalControl. **Idempotent:** ist der Arm schon `RUNNING` + Safety `NORMAL`/`REDUCED` + ExternalControl aktiv, meldet `prepare` sofort `success=True` **ohne** `robot_state_helper` (Demo läuft auch beim wiederholten Start durch). Sonst delegiert es; `robot_state_helper` überspringt erledigte Schritte selbst. |
| `~/recover` | `[pstop-wait] {RUNNING, stop_program, play_program}` | Nach Safety-Violation: Programm stoppen, `RUNNING` wiederherstellen, neu starten (UR-Empfehlung nach einem Stop). Safety-Handling macht der Helper. |
| `~/ensure_ready` | wie `recover` | `SetMode` macht ohnehin „whatever it takes" → identisch zu `recover` (inkl. CB3-Wartezeit). |
| `~/power_off` | `{POWER_OFF, stop_program}` | Arm sicher abschalten. |

Voller Servicename = Node-Name vorangestellt, z. B. `/a200_0553/manipulators/ur_state_manager/prepare`.

### Auto-Recovery bei spätem Einschalten des Arms (`auto_recover`, Default an)

Wird der UR **erst nach dem Boot** bestromt, läuft ExternalControl nicht an: Teach-Panel
„Paused", Arm ohne Feedback (in RViz „liegend"), Greifer stromlos. Ein Watcher-Timer
erkennt den Zustand **„bestromt, aber ExternalControl aus"** (`robot_mode` ∈
{`POWER_ON`,`IDLE`,`RUNNING`} und `robot_program_running=False`) und ruft selbsttätig
`recover`. Bewusst `recover` (nicht `prepare`): `stop_program=True` erzwingt einen
**frischen** ExternalControl-Start → der Treiber sync't `Command=Ist` → **kein
Positionssprung/Protective-Stop**, anders als ein blosses `prepare`/`play`, das den
Paused-Stand mit veraltetem Command fortsetzt. Danach zieht die `rg6_control`-Programm-
Flanke Tool-Power + Prime automatisch nach — spätes Einschalten braucht so **keinen
manuellen Handgriff** mehr. Nur bestromte Zustände werden angefasst; `POWER_OFF`/
`BOOTING`/`BACKDRIVE` bleiben unberührt. Abschaltbar mit `auto_recover:=false`.

> **CB3-Sonderfall:** `robot_state_helper` löst einen Protective-Stop *sofort*, der CB3
> verweigert das aber < 5 s nach dem Auslösen. Deshalb liest `recover`/`ensure_ready`
> vorher `dashboard_client/get_safety_mode` und wartet bei `PROTECTIVE_STOP` kurz
> (`protective_stop_wait`, Default 6 s), **bevor** das Goal rausgeht.

## Controller pro Anwendungsfall umschalten (`controller_mode_manager`)

Zweiter Node + Launch zum **Laufzeit-Umschalten der Arm-Controller**. Idee: EIN
`controller_manager` hostet alle Controller; die sich gegenseitig ausschließenden
**Command-Controller** liegen meist **inaktiv** und werden per `switch_controller`
aktiviert. Der Basis-Satz (von Clearpath gespawnt: `joint_state_broadcaster`,
`arm_0_joint_trajectory_controller`, `io_and_status_controller`) bleibt unangetastet.

`arm_controllers.launch.py` lädt die Extra-Controller `--inactive` (aus
`config/extra_controllers.yaml`) und startet den Mode-Manager:

```bash
ros2 launch ur_state_manager arm_controllers.launch.py
```

Modi (Default; `mode_names`/`mode_controllers`-Parameter überschreibbar) — je ein
`std_srvs/Trigger`-Service:

| Service | aktiviert | Zweck |
|---|---|---|
| `~/mode/trajectory` | `arm_0_joint_trajectory_controller` | MoveIt/Trajektorien (Default) |
| `~/mode/freedrive` | `freedrive_mode_controller` | Hand-Führen / Trajektorien-Recording |
| `~/mode/forward_position` | `forward_position_controller` | direkte Positions-Streams |
| `~/mode/forward_velocity` | `forward_velocity_controller` | direkte Geschwindigkeits-Streams |
| `~/mode/passthrough` | `passthrough_trajectory_controller` | Trajektorien-Streaming |
| `~/release` | – | alle Command-Controller deaktivieren (Arm frei) |
| `~/active` | – | aktiven Command-Controller melden |

Ein Umschalten aktiviert den Zielcontroller und **deaktiviert** die anderen
Command-Controller, die gerade aktiv sind (über `switch_controller`, `STRICT`).
Zusätzlich lädt das Launch die Broadcaster `force_torque_sensor_broadcaster`,
`tcp_pose_broadcaster`, `speed_scaling_state_broadcaster` **aktiv** (kollidieren
nicht). Die Controller-Params in `config/extra_controllers.yaml` sind 1:1 aus
`ur_robot_driver/config/ur_controllers.yaml` übernommen (tf_prefix `arm_0_`).

```bash
# z. B. zum Trajektorien-Recording in FreeDrive
ros2 service call /a200_0553/manipulators/ur_controller_mode_manager/mode/freedrive std_srvs/srv/Trigger
# ... aufzeichnen ... dann zurück:
ros2 service call /a200_0553/manipulators/ur_controller_mode_manager/mode/trajectory std_srvs/srv/Trigger
```

### Recovery-Logik (`~/recover`)

Das komplette Safety-Handling steckt jetzt im `robot_state_helper` (aufgerufen aus
`recover`/`ensure_ready` über ein `SetMode{RUNNING, stop_program, play_program}`-Goal):

| `safety_mode` | Behandlung durch `robot_state_helper` |
|---|---|
| `NORMAL` / `REDUCED` | Keine Violation → direkt Mode-Transition bis `RUNNING`. |
| `PROTECTIVE_STOP` | `unlock_protective_stop` (der **Adapter** wartet vorher ≥ 6 s, s. o.). |
| `SAFEGUARD_STOP` | Wird durch physisches Reset aufgehoben; die Transition wartet darauf. |
| `VIOLATION` / `FAULT` | `restart_safety` (Arm schaltet ab), danach Hochfahren bis `RUNNING`. |
| `*_EMERGENCY_STOP` | Nicht per Software lösbar → Fehler im Result, E-Stop physisch entriegeln. |

## Voraussetzungen

- Der `ur_robot_driver` läuft und ist mit dem UR5 verbunden.
- Der `io_and_status_controller` ist geladen/aktiv (auf a200-0553 ohnehin für den RG6 nötig).
  Der `robot_state_helper` abonniert daraus `robot_mode`/`safety_mode`/`robot_program_running`
  und ruft `resend_robot_program`.
- **Der `robot_state_helper`-Node läuft.** Clearpath startet ihn **nicht** mit; dieses Launch
  startet ihn deshalb selbst (Node-Name `ur_robot_state_helper` im manipulators-Namespace).
  Er öffnet eine **eigene** Primary-Interface-Verbindung zu `robot_ip:30001` für
  `power_on`/`brake_release`/`unlock_protective_stop`. Prüfen mit
  `ros2 action list | grep set_mode` bzw. `ros2 pkg executables ur_robot_driver | grep robot_state_helper`.
- **Der `dashboard_client`-Node läuft.** Clearpath startet ihn im headless-Setup **nicht**
  mit – `robot_state_helper` braucht daraus `restart_safety`/`play` (CB3), der Adapter
  `get_safety_mode`. Dieses Launch startet ihn deshalb standardmäßig selbst mit
  (`start_dashboard_client:=true`). Alternativ manuell:

  ```bash
  ros2 run ur_robot_driver dashboard_client --ros-args \
    -r __ns:=/a200_0553/manipulators \
    -p robot_ip:=192.168.131.40
  ```

  Prüfen mit `ros2 service list | grep dashboard`.

  > Auf a200-0553 startet der `husky-custom-setup`-Installer den `dashboard_client`
  > optional als eigenen Boot-Service (`ur-dashboard.service`). Läuft er bereits darüber,
  > dieses Launch mit `start_dashboard_client:=false` starten, damit nicht zwei
  > Dashboard-Clients gleichzeitig auf Port 29999 verbinden.
- `headless_mode: true` (Clearpath-Default auf a200-0553) → `robot_state_helper` sendet
  ExternalControl per `io_and_status_controller/resend_robot_program`. Bei
  `headless_mode: false` benutzt er stattdessen den Dashboard-`play`.

## Parameter

### `ur_state_manager` (Adapter)

| Parameter | Default | Bedeutung |
|---|---|---|
| `set_mode_action` | `/a200_0553/manipulators/ur_robot_state_helper/set_mode` | Action-Name des `robot_state_helper`. |
| `dashboard_ns` | `/a200_0553/manipulators/dashboard_client` | Für `get_safety_mode` + `get_robot_mode` (CB3-Wartezeit / idempotenter `prepare`-Vorcheck). |
| `io_status_ns` | `/a200_0553/manipulators/io_and_status_controller` | Für `robot_program_running` (ExternalControl aktiv? → idempotenter `prepare`-Vorcheck). |
| `service_timeout` | `10.0` | Timeout beim Warten auf Action-Server/Service (s). |
| `action_timeout` | `120.0` | Max. Wartezeit auf das `SetMode`-Ergebnis (Mode-Transition). |
| `protective_stop_wait` | `6.0` | Wartezeit vor dem `SetMode`-Goal bei `PROTECTIVE_STOP` (CB3 ≥ 5 s). |
| `auto_recover` | `true` | Watcher, der nach spätem Einschalten selbsttätig `recover` ausführt (s. o.). `false` → aus. |
| `auto_recover_period` | `5.0` | Prüfintervall des Watchers (s). |
| `auto_recover_settle` | `2` | So viele konsistente „muss recovern"-Beobachtungen vor dem Handeln (entprellt Boot-/`prepare`-Übergänge). |

### `ur_robot_state_helper` (aus `ur_robot_driver`, per Launch gestartet)

| Parameter | Default (Launch) | Bedeutung |
|---|---|---|
| `robot_ip` | `192.168.131.40` | UR-Control-Box (Primary-Interface Port 30001). |
| `headless_mode` | `true` | `true` → ExternalControl via `resend_robot_program`, sonst `play`. |

## Build

```bash
git clone https://github.com/CLAIRLab-HAW/ur-state-manager.git
cd ur-state-manager
colcon build --packages-select ur_state_manager
source install/setup.bash
```

`ur_dashboard_msgs` kommt mit dem `ur_robot_driver`-Stack (auf der a200-0553 vorhanden).

> Auf a200-0553 erledigt das der `husky-custom-setup`-Installer optional automatisch:
> er klont+baut dieses Repo und installiert `ur-state-manager.service` (startet den
> Manager beim Boot, `start_dashboard_client:=false`, da der `dashboard_client` über
> `ur-dashboard.service` läuft).

## Starten

```bash
# startet dashboard_client + robot_state_helper + ur_state_manager (Adapter)
# robot_ip 192.168.131.40, headless_mode:=true
ros2 launch ur_state_manager ur_state_manager.launch.py

# falls der dashboard_client schon anderweitig laeuft:
ros2 launch ur_state_manager ur_state_manager.launch.py start_dashboard_client:=false

# oder nur den Adapter direkt (setzt voraus, dass robot_state_helper schon laeuft):
ros2 run ur_state_manager state_manager --ros-args \
  -r __ns:=/a200_0553/manipulators \
  -p set_mode_action:=/a200_0553/manipulators/ur_robot_state_helper/set_mode \
  -p dashboard_ns:=/a200_0553/manipulators/dashboard_client
```

## Benutzen

```bash
# Arm einsatzbereit machen (power on + brakes lösen + ExternalControl)
ros2 service call /a200_0553/manipulators/ur_state_manager/prepare std_srvs/srv/Trigger

# Nach Kollision / Protective-Stop wieder bereit machen
ros2 service call /a200_0553/manipulators/ur_state_manager/recover std_srvs/srv/Trigger

# „mach einfach bereit, egal welcher Zustand"
ros2 service call /a200_0553/manipulators/ur_state_manager/ensure_ready std_srvs/srv/Trigger

# sicher abschalten
ros2 service call /a200_0553/manipulators/ur_state_manager/power_off std_srvs/srv/Trigger
```

Jede Antwort liefert `success` (bool) und `message` (string) mit Klartext-Status.
Es läuft immer nur **ein** Vorgang gleichzeitig (parallele Aufrufe werden mit
`success=false` abgewiesen).

## Hinweis zur Einbindung über `robot.yaml`

Damit der Workspace gefunden wird, muss er – wie `rg6_control` – unter
`system.ros2.workspaces` in der `robot.yaml` eingetragen sein. Der Node lässt sich dann
analog zu `rg6_bringup.launch.py` über `platform.extras.launch` mitstarten.
