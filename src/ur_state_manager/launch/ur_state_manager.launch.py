#!/usr/bin/env python3
"""Startet die Arm-State-Verwaltung fuer die a200-0553.

Drei Nodes:
  * dashboard_client (ur_robot_driver)   - Dashboard-Services (TCP 29999). Clearpath
    bringt ihn im headless-Setup nicht mit; robot_state_helper braucht daraus
    restart_safety/play, der Adapter get_safety_mode. Default: mitstarten.
  * robot_state_helper (ur_robot_driver) - die eigentliche Mode-/Safety-Recovery.
    Er oeffnet eine eigene Primary-Interface-Verbindung (robot_ip:30001) fuer
    power_on/brake_release/unlock_protective_stop und nutzt relative Clients
    dashboard_client/{restart_safety,play} + io_and_status_controller/
    resend_robot_program sowie die *_mode-Topics -> laeuft daher im
    manipulators-Namespace, damit alle relativen Namen passen.
  * ur_state_manager (dieses Paket)      - duenner Adapter: haelt die gewohnte
    Trigger-API (prepare/recover/ensure_ready/power_off) und delegiert an die
    SetMode-Action des robot_state_helper.

Defaults passen zum UR5 (CB3) auf a200-0553 (headless_mode, manipulators-Namespace).
Per Launch-Argument ueberschreibbar.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

NS = "/a200_0553/manipulators"
ROBOT_IP = "192.168.131.40"


def generate_launch_description():
    dashboard_ns = LaunchConfiguration("dashboard_ns")
    headless_mode = LaunchConfiguration("headless_mode")
    start_dashboard_client = LaunchConfiguration("start_dashboard_client")
    robot_ip = LaunchConfiguration("robot_ip")

    return LaunchDescription([
        DeclareLaunchArgument(
            "dashboard_ns", default_value=f"{NS}/dashboard_client",
            description="Namespace des ur_robot_driver Dashboard-Clients "
                        "(fuer get_safety_mode im Adapter)."),
        DeclareLaunchArgument(
            "headless_mode", default_value="true",
            description="true -> ExternalControl via resend_robot_program "
                        "(Clearpath-Default auf a200-0553)."),
        DeclareLaunchArgument(
            "start_dashboard_client", default_value="true",
            description="Den ur_robot_driver dashboard_client mitstarten "
                        "(noetig, da Clearpath ihn nicht mitbringt)."),
        DeclareLaunchArgument(
            "robot_ip", default_value=ROBOT_IP,
            description="IP der UR-Control-Box (Dashboard-Server Port 29999)."),

        # Dashboard-Client aus ur_robot_driver. Node-Name 'dashboard_client' im
        # manipulators-Namespace -> Services landen unter
        # /a200_0553/manipulators/dashboard_client/* (= Default dashboard_ns).
        Node(
            package="ur_robot_driver",
            executable="dashboard_client",
            name="dashboard_client",
            namespace=NS,
            output="screen",
            emulate_tty=True,
            condition=IfCondition(start_dashboard_client),
            parameters=[{"robot_ip": robot_ip}],
        ),

        # Offizielle Mode-/Safety-Recovery. Muss im manipulators-Namespace laufen,
        # damit seine relativen Clients dashboard_client/* und io_and_status_controller/*
        # sowie die *_mode-Topics aufloesen. headless_mode -> ExternalControl via
        # resend_robot_program statt Dashboard-play.
        Node(
            package="ur_robot_driver",
            executable="robot_state_helper",
            name="ur_robot_state_helper",
            namespace=NS,
            output="screen",
            emulate_tty=True,
            parameters=[{
                "robot_ip": robot_ip,
                "headless_mode": headless_mode,
            }],
        ),

        # Duenner Adapter: gewohnte Trigger-API -> SetMode-Action des Helpers.
        Node(
            package="ur_state_manager",
            executable="state_manager",
            name="ur_state_manager",
            namespace=NS,
            output="screen",
            parameters=[{
                "set_mode_action": f"{NS}/ur_robot_state_helper/set_mode",
                "dashboard_ns": dashboard_ns,
            }],
        ),
    ])
