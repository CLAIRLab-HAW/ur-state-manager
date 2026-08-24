#!/usr/bin/env python3
"""Loads the additional arm controllers into the manipulators controller_manager
and starts the controller mode manager (a200-0553).

  1. A wrapper loads the broadcasters (ft/tcp_pose/speed_scaling) ACTIVE and the
     command controllers (freedrive/forward/passthrough) --inactive, both from
     config/extra_controllers.yaml (type + params via --param-file) -- but ONLY
     those that are not loaded yet (see below).
  2. ur_controller_mode_manager: switches between the modes at runtime via a
     Trigger service (trajectory/freedrive/forward_*/passthrough).

The base set (joint_state_broadcaster, arm_0_joint_trajectory_controller, io_and_status_controller) is spawned by
Clearpath from robot.yaml and is NOT touched here.

WHY ONE WRAPPER INSTEAD OF SEVEN PARALLEL spawner NODES
-------------------------------------------------------
Reproduced three times on 2026-07-29: a 'systemctl restart
clearpath-custom-ur-state-manager' against an **already populated** arm CM made
the spawners load the ur_controllers controllers AGAIN -- the ros2_control_node
died with SIGSEGV in libur_controllers.so during the lifecycle transition, and
the arm CM was dead afterwards (list_controllers without an answer).  On a
restart of clearpath-manipulators this does NOT happen: there the CM is fresh
and empty, the spawners load for the first time -- and that works even with an
unpowered arm.

So the difference is the **re-spawning of already loaded** controllers, not the power state. (Gating on the hardware
state does not help: the component reports 'label=active' even at POWER_OFF -- 'active' only means "the driver reads
via RTDE".)

Hence: first query 'ros2 control list_controllers', then spawn only the missing ones. If everything is already there,
nothing happens -- the restart becomes a no-op instead of a crash. Sequential rather than parallel, so that the query
does not collide with our own spawns.
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

NS = "/a200_0553/manipulators"
CONTROLLER_MANAGER = NS + "/controller_manager"

# Broadcasters loaded active (they do not collide with the jtc).
BROADCASTERS = ["force_torque_sensor_broadcaster", "tcp_pose_broadcaster", "speed_scaling_state_broadcaster"]

# Command controllers that are loaded --inactive (order does not matter). Has to match the type entries in
# config/extra_controllers.yaml.
COMMAND_CONTROLLERS = [
    "forward_position_controller",
    "forward_velocity_controller",
    "freedrive_mode_controller",
    "passthrough_trajectory_controller",
]

# $1 = path to extra_controllers.yaml (handed over by the launch file as a substitution).
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
