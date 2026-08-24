#!/usr/bin/env python3
"""Laedt die zusaetzlichen Arm-Controller in den manipulators-controller_manager
und startet den Controller-Mode-Manager (a200-0553).

  1. Ein Wrapper laedt die Broadcaster (ft/tcp_pose/speed_scaling) AKTIV und die
     Command-Controller (freedrive/forward/passthrough) --inactive, beide aus
     config/extra_controllers.yaml (Typ + Params via --param-file) -- aber NUR
     die, die noch nicht geladen sind (s. u.).
  2. ur_controller_mode_manager: schaltet zur Laufzeit per Trigger-Service zwischen
     den Modi um (trajectory/freedrive/forward_*/passthrough).

Der Basis-Satz (joint_state_broadcaster, arm_0_joint_trajectory_controller, io_and_status_controller) wird von Clearpath
aus der robot.yaml gespawnt und hier NICHT angefasst.

WARUM EIN WRAPPER STATT SIEBEN PARALLELER spawner-Nodes
-------------------------------------------------------
Am 2026-07-29 dreimal reproduziert: ein 'systemctl restart
clearpath-custom-ur-state-manager' gegen einen **bereits bestueckten** Arm-CM
liess die Spawner die ur_controllers-Controller ERNEUT laden -- der
ros2_control_node starb mit SIGSEGV in libur_controllers.so waehrend der
Lifecycle-Transition, der Arm-CM war danach tot (list_controllers ohne Antwort).
Beim Neustart von clearpath-manipulators passiert das NICHT: dort ist der CM
frisch und leer, die Spawner laden zum ersten Mal -- das funktioniert auch bei
stromlosem Arm.

Der Unterschied ist also das **Re-Spawnen bereits geladener** Controller, nicht der Stromzustand. (Ein Gate auf den
Hardware-Zustand hilft nicht: die Komponente meldet 'label=active' auch bei POWER_OFF -- 'active' heisst nur "Treiber
liest ueber RTDE".)

Deshalb: erst 'ros2 control list_controllers' abfragen, dann nur die fehlenden spawnen. Ist alles schon da, passiert
nichts -- der Restart wird zum No-op statt zum Absturz. Sequenziell statt parallel, damit die Abfrage nicht mit den
eigenen Spawns kollidiert.
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

NS = "/a200_0553/manipulators"
CONTROLLER_MANAGER = NS + "/controller_manager"

# Aktiv geladene Broadcaster (kollidieren nicht mit dem jtc).
BROADCASTERS = ["force_torque_sensor_broadcaster", "tcp_pose_broadcaster", "speed_scaling_state_broadcaster"]

# Command-Controller, die --inactive geladen werden (Reihenfolge egal). Muss zu den Typ-Eintraegen in
# config/extra_controllers.yaml passen.
COMMAND_CONTROLLERS = [
    "forward_position_controller",
    "forward_velocity_controller",
    "freedrive_mode_controller",
    "passthrough_trajectory_controller",
]

# $1 = Pfad zur extra_controllers.yaml (vom Launch als Substitution uebergeben).
_LOAD_SCRIPT = r"""
set -u
CM="__CM__"
PARAMS="$1"
ACTIVE="__ACTIVE__"
INACTIVE="__INACTIVE__"
TAG="arm_controllers"

# 1) Auf den controller_manager warten (Boot: der CM kommt erst hoch).
LOADED=""
for i in $(seq 1 60); do
    if out="$(ros2 control list_controllers -c "$CM" 2>/dev/null)"; then
        LOADED="$(printf '%s\n' "$out" | awk '{print $1}')"
        break
    fi
    sleep 2
done
if [ -z "$LOADED" ] && ! ros2 control list_controllers -c "$CM" >/dev/null 2>&1; then
    echo "$TAG: controller_manager $CM nicht erreichbar - keine Extra-Controller geladen." >&2
    exit 1
fi

already() { printf '%s\n' "$LOADED" | grep -qx "$1"; }

# 2) Nur fehlende Controller spawnen. Ein erneutes Laden eines bereits
#    geladenen ur_controllers-Controllers bringt den CM zum Absturz (s. Docstring).
rc=0
spawn() {
    name="$1"; shift
    if already "$name"; then
        echo "$TAG: $name bereits geladen -> uebersprungen"
        return 0
    fi
    echo "$TAG: spawne $name $*"
    ros2 run controller_manager spawner "$name" "$@" \
        --param-file "$PARAMS" -c "$CM" --controller-manager-timeout 60 || rc=1
}

for c in $ACTIVE;   do spawn "$c"; done
for c in $INACTIVE; do spawn "$c" --inactive; done
exit $rc
"""


def generate_launch_description():
    extra_params = PathJoinSubstitution([FindPackageShare("ur_state_manager"), "config", "extra_controllers.yaml"])

    load_controllers = ExecuteProcess(
        cmd=[
            "bash",
            "-c",
            (
                _LOAD_SCRIPT.replace("__CM__", CONTROLLER_MANAGER)
                .replace("__ACTIVE__", " ".join(BROADCASTERS))
                .replace("__INACTIVE__", " ".join(COMMAND_CONTROLLERS))
            ),
            "arm_controllers",
            extra_params,
        ],
        name="arm_controllers_loader",
        output="screen",
    )

    mode_manager = Node(
        package="ur_state_manager",
        executable="controller_mode_manager",
        name="ur_controller_mode_manager",
        namespace=NS,
        output="screen",
    )

    return LaunchDescription([mode_manager, load_controllers])
