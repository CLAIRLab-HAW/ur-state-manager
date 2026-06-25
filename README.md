# ur_arm_manager

ROS2-Node, der einen **bereits verbundenen** UR5 (CB3) auf der `a200-0553` verwaltet –
über den `ur_robot_driver` **Dashboard-Client** (TCP 29999 auf der UR-Control-Box) und
den `io_and_status_controller`.

Er deckt zwei Aufgaben ab:

1. **Einsatzbereit machen** – `power_on` → `brake_release` → ExternalControl starten.
2. **Recovery nach Safety-Violation** – Arm ist nach Kollision / Protective-Stop in den
   Lock-/Stop-State gefallen und soll wieder bereit werden.

> Der Node **bewegt den Arm nicht** und sendet keine Trajektorien. Er stellt nur sicher,
> dass der Arm bestromt, Bremsen gelöst und ROS-kontrollierbar (ExternalControl aktiv) ist.

## Angebotene Services (alle `std_srvs/Trigger`)

| Service | Wirkung |
|---|---|
| `~/prepare` | `power_on` + `brake_release` + ExternalControl. Überspringt Schritte, die schon erledigt sind (z. B. wenn der Arm bereits `RUNNING` ist). |
| `~/recover` | Liest `safety_mode` und behandelt ihn (siehe Tabelle unten), danach automatisch `prepare()`. |
| `~/ensure_ready` | Liegt eine Safety-Violation vor → `recover`, sonst `prepare`. Der bequeme „mach den Arm einfach bereit"-Aufruf. |
| `~/power_off` | Arm sicher abschalten (`power_off`). |

Voller Servicename = Node-Name vorangestellt, z. B. `/ur_arm_manager/prepare`.

### Recovery-Logik (`~/recover`)

| `safety_mode` | Behandlung |
|---|---|
| `NORMAL` / `REDUCED` | Keine Violation → direkt `prepare()`. |
| `PROTECTIVE_STOP` | ≥ 6 s warten (CB3-Pflicht, < 5 s nicht lösbar), `unlock_protective_stop`, dann `prepare()`. |
| `SAFEGUARD_STOP` | Schutzeinrichtung muss **physisch** zurückgesetzt werden; wartet auf Aufhebung, dann `prepare()`. |
| `VIOLATION` / `FAULT` | `restart_safety` (Arm schaltet ab), `close_safety_popup`, dann voller `prepare()`. |
| `*_EMERGENCY_STOP` | Nicht per Software lösbar → Fehlermeldung, E-Stop physisch entriegeln. |

## Voraussetzungen

- Der `ur_robot_driver` läuft und ist mit dem UR5 verbunden.
- Der `io_and_status_controller` ist geladen/aktiv (auf a200-0553 ohnehin für den RG6 nötig).
- **Der `dashboard_client`-Node läuft.** Clearpath startet ihn im headless-Setup **nicht**
  mit – `power_on`/`brake_release`/`unlock_protective_stop`/`restart_safety`/`get_*_mode`
  gibt es aber nur dort. Dieses Launch startet ihn deshalb standardmäßig selbst mit
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
- `headless_mode: true` (Clearpath-Default auf a200-0553) → ExternalControl wird per
  `io_and_status_controller/resend_robot_program` (neu) gesendet. Bei `headless_mode: false`
  wird stattdessen optional `load_program` + `play` benutzt (Parameter `program_name`).

## Parameter

| Parameter | Default | Bedeutung |
|---|---|---|
| `dashboard_ns` | `/a200_0553/manipulators/dashboard_client` | Namespace des Dashboard-Clients. |
| `io_status_ns` | `/a200_0553/manipulators/io_and_status_controller` | Namespace für `resend_robot_program`. |
| `headless_mode` | `true` | `true` → ExternalControl via `resend_robot_program`. |
| `program_name` | `""` | Nur bei `headless_mode:=false`: `.urp` vor `play` laden. |
| `service_timeout` | `10.0` | Timeout je Dashboard-Call (s). |
| `protective_stop_wait` | `6.0` | Wartezeit vor `unlock_protective_stop` (CB3 ≥ 5 s). |
| `mode_timeout` | `30.0` | Max. Wartezeit auf Mode-Wechsel (z. B. → `RUNNING`). |
| `mode_poll_interval` | `0.5` | Poll-Intervall für `get_robot_mode`/`get_safety_mode`. |

## Build

In einem ROS2-Workspace (oder diesen `src`-Baum in einen vorhandenen Workspace einbinden):

```bash
cd ur-arm-manager
colcon build --packages-select ur_arm_manager
source install/setup.bash
```

`ur_dashboard_msgs` kommt mit dem `ur_robot_driver`-Stack (auf der a200-0553 vorhanden).

## Starten

```bash
# startet dashboard_client (robot_ip 192.168.131.40) + ur_arm_manager
ros2 launch ur_arm_manager ur_arm_manager.launch.py

# falls der dashboard_client schon anderweitig laeuft:
ros2 launch ur_arm_manager ur_arm_manager.launch.py start_dashboard_client:=false

# oder nur den Manager direkt:
ros2 run ur_arm_manager arm_manager --ros-args \
  -p dashboard_ns:=/a200_0553/manipulators/dashboard_client \
  -p io_status_ns:=/a200_0553/manipulators/io_and_status_controller
```

## Benutzen

```bash
# Arm einsatzbereit machen (power on + brakes lösen + ExternalControl)
ros2 service call /ur_arm_manager/prepare std_srvs/srv/Trigger

# Nach Kollision / Protective-Stop wieder bereit machen
ros2 service call /ur_arm_manager/recover std_srvs/srv/Trigger

# „mach einfach bereit, egal welcher Zustand"
ros2 service call /ur_arm_manager/ensure_ready std_srvs/srv/Trigger

# sicher abschalten
ros2 service call /ur_arm_manager/power_off std_srvs/srv/Trigger
```

Jede Antwort liefert `success` (bool) und `message` (string) mit Klartext-Status.
Es läuft immer nur **ein** Vorgang gleichzeitig (parallele Aufrufe werden mit
`success=false` abgewiesen).

## Hinweis zur Einbindung über `robot.yaml`

Damit der Workspace gefunden wird, muss er – wie `rg6_control` – unter
`system.ros2.workspaces` in der `robot.yaml` eingetragen sein. Der Node lässt sich dann
analog zu `rg6_bringup.launch.py` über `platform.extras.launch` mitstarten.
