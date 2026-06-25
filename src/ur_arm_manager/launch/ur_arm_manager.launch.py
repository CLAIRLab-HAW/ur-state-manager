#!/usr/bin/env python3
"""Startet den ur_arm_manager-Node fuer die a200-0553.

Optional wird der ur_robot_driver Dashboard-Client mitgestartet (Default: an),
weil Clearpath ihn im headless-Setup nicht mitbringt - ohne ihn gibt es keine
power_on/brake_release/unlock_protective_stop-Services.

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
    io_status_ns = LaunchConfiguration("io_status_ns")
    headless_mode = LaunchConfiguration("headless_mode")
    start_dashboard_client = LaunchConfiguration("start_dashboard_client")
    robot_ip = LaunchConfiguration("robot_ip")

    return LaunchDescription([
        DeclareLaunchArgument(
            "dashboard_ns", default_value=f"{NS}/dashboard_client",
            description="Namespace des ur_robot_driver Dashboard-Clients."),
        DeclareLaunchArgument(
            "io_status_ns", default_value=f"{NS}/io_and_status_controller",
            description="Namespace des io_and_status_controller (resend_robot_program)."),
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

        Node(
            package="ur_arm_manager",
            executable="arm_manager",
            name="ur_arm_manager",
            output="screen",
            parameters=[{
                "dashboard_ns": dashboard_ns,
                "io_status_ns": io_status_ns,
                "headless_mode": headless_mode,
            }],
        ),
    ])
