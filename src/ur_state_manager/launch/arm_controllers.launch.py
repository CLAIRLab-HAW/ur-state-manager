#!/usr/bin/env python3
"""Laedt die zusaetzlichen Arm-Controller in den manipulators-controller_manager
und startet den Controller-Mode-Manager (a200-0553).

  1. controller_manager-spawner laedt die Broadcaster (ft/tcp_pose/speed_scaling)
     AKTIV und die Command-Controller (freedrive/forward/passthrough) --inactive,
     beide aus config/extra_controllers.yaml (Typ + Params via --param-file).
  2. ur_controller_mode_manager: schaltet zur Laufzeit per Trigger-Service zwischen
     den Modi um (trajectory/freedrive/forward_*/passthrough).

Der Basis-Satz (joint_state_broadcaster, arm_0_joint_trajectory_controller,
io_and_status_controller) wird von Clearpath aus der robot.yaml gespawnt und hier
NICHT angefasst.
"""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

NS = "/a200_0553/manipulators"
CONTROLLER_MANAGER = NS + "/controller_manager"

# Aktiv geladene Broadcaster (kollidieren nicht mit dem jtc).
BROADCASTERS = [
    "force_torque_sensor_broadcaster",
    "tcp_pose_broadcaster",
    "speed_scaling_state_broadcaster",
]

# Command-Controller, die --inactive geladen werden (Reihenfolge egal).
# Muss zu den Typ-Eintraegen in config/extra_controllers.yaml passen.
COMMAND_CONTROLLERS = [
    "forward_position_controller",
    "forward_velocity_controller",
    "freedrive_mode_controller",
    "passthrough_trajectory_controller",
]


def generate_launch_description():
    extra_params = PathJoinSubstitution(
        [FindPackageShare("ur_state_manager"), "config", "extra_controllers.yaml"])

    # Broadcaster AKTIV laden (Typ via --param-file).
    load_active = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                broadcaster_name,
                "--param-file", extra_params,
                "-c", CONTROLLER_MANAGER,
                "--controller-manager-timeout", "60",
            ],
            output="screen",
        ) for broadcaster_name in BROADCASTERS
    ]

    # Command-Controller in einem Rutsch INAKTIV laden (Typ via --param-file).
    load_inactive = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                controller_name,
                "--inactive",
                "--param-file", extra_params,
                "-c", CONTROLLER_MANAGER,
                "--controller-manager-timeout", "60",
            ],
            output="screen",
        ) for controller_name in COMMAND_CONTROLLERS
    ]


    mode_manager = Node(
        package="ur_state_manager",
        executable="controller_mode_manager",
        name="ur_controller_mode_manager",
        namespace=NS,
        output="screen",
    )

    return LaunchDescription([*load_active, *load_inactive, mode_manager])
