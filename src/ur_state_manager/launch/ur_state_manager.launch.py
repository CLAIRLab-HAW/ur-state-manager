#!/usr/bin/env python3
"""Starts the arm state management for the a200-0553.

Three nodes:
  * dashboard_client (ur_robot_driver)   - dashboard services (TCP 29999). In the
    headless setup Clearpath does not bring it along; robot_state_helper needs
    restart_safety/play from it, the adapter needs get_safety_mode. Default:
    start it along.
  * robot_state_helper (ur_robot_driver) - the actual mode/safety recovery.
    It opens a primary-interface connection of its own (robot_ip:30001) for
    power_on/brake_release/unlock_protective_stop and uses the relative clients
    dashboard_client/{restart_safety,play} + io_and_status_controller/
    resend_robot_program as well as the *_mode topics -> it therefore runs in
    the manipulators namespace, so that all relative names resolve.
  * ur_state_manager (this package)      - thin adapter: keeps the familiar
    Trigger API (prepare/recover/ensure_ready/power_off) and delegates to the
    SetMode action of the robot_state_helper.

The defaults fit the UR5 (CB3) on the a200-0553 (headless_mode, manipulators namespace). Overridable by launch
argument.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

NS = "/a200_0553/manipulators"
ROBOT_IP = "192.168.131.40"


def generate_launch_description():
    dashboard_ns = LaunchConfiguration("dashboard_ns")
    headless_mode = LaunchConfiguration("headless_mode")
    start_dashboard_client = LaunchConfiguration("start_dashboard_client")
    robot_ip = LaunchConfiguration("robot_ip")
    load_arm_controllers = LaunchConfiguration("load_arm_controllers")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "dashboard_ns",
                default_value=f"{NS}/dashboard_client",
                description="Namespace of the ur_robot_driver dashboard client (for get_safety_mode in the adapter).",
            ),
            DeclareLaunchArgument(
                "headless_mode",
                default_value="true",
                description="true -> ExternalControl via resend_robot_program "
                "(the Clearpath default on the a200-0553).",
            ),
            DeclareLaunchArgument(
                "start_dashboard_client",
                default_value="true",
                description="Start the ur_robot_driver dashboard_client along with it "
                "(necessary, because Clearpath does not bring it along).",
            ),
            DeclareLaunchArgument(
                "robot_ip",
                default_value=ROBOT_IP,
                description="IP of the UR control box (dashboard server, port 29999).",
            ),
            DeclareLaunchArgument(
                "load_arm_controllers",
                default_value="true",
                description="arm_controllers.launch.py (extra controller + mode manager). Deliberately here and not in "
                "its own systemd unit: same workspace, same user, same dependencies and identical lifecycle.",
            ),
            # Extra controllers + mode manager. NOT doable via robot.yaml: Clearpath's spawn loop
            # (clearpath_manipulators/launch/control.launch.py) spawns every control.yaml node whose NAME contains
            # 'controller' -- and always ACTIVE. The broadcasters (…_broadcaster) would therefore never be loaded, the
            # command controllers on the other hand active and in collision with the arm_0_joint_trajectory_controller.
            # Hence a spawner launch of our own.
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare("ur_state_manager"), "launch", "arm_controllers.launch.py"])
                ),
                condition=IfCondition(load_arm_controllers),
            ),
            # Dashboard client from ur_robot_driver. Node name 'dashboard_client' in the manipulators namespace ->
            # the services land under /a200_0553/manipulators/dashboard_client/* (= the default dashboard_ns).
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
            # The official mode/safety recovery. Has to run in the manipulators namespace
            # so that its relative clients dashboard_client/* and io_and_status_controller/*
            # as well as the *_mode topics resolve. headless_mode -> ExternalControl via
            # resend_robot_program instead of the dashboard play.
            #
            # respawn=True: ur_robot_driver 3.7 (jazzy) has an upstream race in
            # RobotStateHelper::setModeExecute -> it uses the SHARED current_goal_handle_
            # (not the local goal_handle parameter) for succeed()/abort(). If a second
            # SetMode goal comes in while the first is still in the wait loop (e.g. a
            # calibration/script prepare and auto_recover at the same time), goal #2
            # overwrites current_goal_handle_; goal #1 then calls succeed() on the already
            # succeeded goal #2 -> rcl_action "invalid transition from SUCCEEDED with event
            # SUCCEED" -> std::terminate -> SIGABRT (exit -6). The state change
            # (POWER_OFF->RUNNING) had already gone through; only the follow-up succeed
            # crashes the node. Without a respawn the helper stays dead -> the set_mode
            # action disappears -> every subsequent prepare/recover fails ("set_mode action
            # not available"). The respawn restarts it after about 2 s; the actual trigger
            # (competing goals) is additionally shut off by auto_recover:=false (no more
            # parallel watcher recoveries).
            Node(
                package="ur_robot_driver",
                executable="robot_state_helper",
                name="ur_robot_state_helper",
                namespace=NS,
                output="screen",
                emulate_tty=True,
                respawn=True,
                respawn_delay=2.0,
                parameters=[{"robot_ip": robot_ip, "headless_mode": headless_mode}],
            ),
            # Thin adapter: the familiar Trigger API -> SetMode action of the helper.
            Node(
                package="ur_state_manager",
                executable="state_manager",
                name="ur_state_manager",
                namespace=NS,
                output="screen",
                parameters=[{"set_mode_action": f"{NS}/ur_robot_state_helper/set_mode", "dashboard_ns": dashboard_ns}],
            ),
        ]
    )
