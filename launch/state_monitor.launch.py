#!/usr/bin/env python3

import launch
import os
import sys

from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    LaunchConfiguration,
    IfElseSubstitution,
    PythonExpression,
    PathJoinSubstitution,
    EnvironmentVariable,
)

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ld = launch.LaunchDescription()

    pkg_name = "mrs_robot_diagnostics"

    this_pkg_path = get_package_share_directory(pkg_name)
    namespace = 'state_monitor'

    # #{ uav_name

    robot_name = LaunchConfiguration('robot_name')

    ld.add_action(DeclareLaunchArgument(
        'robot_name',
        default_value=os.getenv('UAV_NAME', "uav1"),
        description="The uav name used for namespacing.",
    ))

    uav_type = LaunchConfiguration('uav_type')
    ld.add_action(DeclareLaunchArgument(
        'uav_type',
        default_value=os.getenv('UAV_TYPE', "x500"),
        description="The uav type used for selecting platform configuration.",
    ))

    robot_type = LaunchConfiguration('robot_type')
    ld.add_action(DeclareLaunchArgument(
        'robot_type',
        default_value=os.getenv('ROBOT_TYPE', "multirotor"),
        description="The robot type used for selecting platform configuration.",
    ))

    available_sensors = LaunchConfiguration('available_sensors')
    ld.add_action(DeclareLaunchArgument(
        'available_sensors',
        default_value=os.getenv('AVAILABLE_SENSORS', "pixhawk,garmin"),
        description="The available sensors on the platform.",
    ))

    # #} end of custom_config

    # #{ standalone

    standalone = LaunchConfiguration('standalone')

    declare_standalone = DeclareLaunchArgument(
        'standalone',
        default_value='true',
        description='Whether to start a as a standalone or load into an existing container.'
    )

    ld.add_action(declare_standalone)

    # #} end of standalone

    # #{ container_name

    container_name = LaunchConfiguration('container_name')

    declare_container_name = DeclareLaunchArgument(
        'container_name',
        default_value='',
        description='Name of an existing container to load into (if standalone is false)'
    )

    ld.add_action(declare_container_name)

    # #} end of container_name

    # #{ custom_config

    custom_config = LaunchConfiguration('custom_config')

    # this adds the args to the list of args available for this launch files
    # these args can be listed at runtime using -s flag
    # default_value is required to if the arg is supposed to be optional at launch time
    ld.add_action(DeclareLaunchArgument(
        'custom_config',
        default_value="",
        description="Path to the custom configuration file. The path can be absolute, starting with '/' or relative to the current working directory",
    ))

    # behaviour:
    #     custom_config == "" => custom_config: ""
    #     custom_config == "/<path>" => custom_config: "/<path>"
    #     custom_config == "<path>" => custom_config: "$(pwd)/<path>"
    custom_config = IfElseSubstitution(
        condition=PythonExpression(['"', custom_config, '" != "" and ',
                                   'not "', custom_config, '".startswith("/")']),
        if_value=PathJoinSubstitution([EnvironmentVariable('PWD'), custom_config]),
        else_value=custom_config
    )

    # #} end of custom_config

    # #{ use_sim_time

    use_sim_time = LaunchConfiguration('use_sim_time')

    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value=os.getenv('USE_SIM_TIME', "false"),
        description="Should the node subscribe to sim time?",
    ))

    # #} end of custom_config

    # #{ log_level

    ld.add_action(DeclareLaunchArgument(name='log_level', default_value='info'))

    # #} end of log_level

    # #{ state monitor node

    state_monitor_node = ComposableNode(

        package=pkg_name,
        plugin='mrs_robot_diagnostics::state_monitor::StateMonitor',
        namespace=robot_name,
        name='state_monitor',

        parameters=[
            {"available_sensors": available_sensors},
            {"custom_config": custom_config},
            {"robot_name": robot_name},
            {"robot_type": robot_type},
            {"uav_type": uav_type},
            {"use_sim_time": use_sim_time},
            {'config': this_pkg_path + '/config/state_monitor_config.yaml'},
            {'preflight_check_config': this_pkg_path + '/config/preflight_check_config.yaml'},
            # additional parameters for camera plugin 
            {'camera_frame': [robot_name, '/servo_camera/camera_frame']},
            {'camera_info_topic': [robot_name, '/servo_camera/camera_info']},
            {'camera_orientation_topic': [robot_name, '/servo_camera/orientation']},
            {'fcu_frame': [robot_name, '/fcu']},
            {'image_topic': [robot_name, '/servo_camera/image_raw']},
            {'optical_frame': [robot_name, '/servo_camera/optical_frame']},
        ],

        remappings=[

            # publishers
            ("~/collision_avoidance_info_out", "~/collision_avoidance_info"),
            ("~/control_info_out", "~/control_info"),
            ("~/general_robot_info_out", "~/general_robot_info"),
            ("~/sensor_info_out", "~/sensor_info"),
            ("~/state_estimation_info_out", "~/state_estimation_info"),
            ("~/system_health_info_out", "~/system_health_info"),
            ("~/uav_info_out", "~/uav_info"),
            ("~/uav_state_out", "~/uav_state"),
            # subscribers
            ("~/battery_state_in", "hw_api/battery_state"),
            ("~/control_manager_diagnostics_in", "control_manager/diagnostics"),
            ("~/control_manager_heading_in", "control_manager/heading"),
            ("~/control_manager_thrust_in", "control_manager/thrust"),
            ("~/constraint_manager_diagnostics_in", "constraint_manager/diagnostics"),
            ("~/estimation_diagnostics_in", "estimation_manager/diagnostics"),
            ("~/gain_manager_diagnostics_in", "gain_manager/diagnostics"),
            ("~/hw_api_capabilities_in", "hw_api/capabilities"),
            ("~/hw_api_distance_sensor_in", "hw_api/distance_sensor"),
            ("~/hw_api_gnss_in", "hw_api/gnss"),
            ("~/hw_api_gnss_status_in", "hw_api/gnss_status"),
            ("~/hw_api_imu_in", "hw_api/imu"),
            ("~/hw_api_mag_heading_in", "hw_api/mag_heading"),
            ("~/hw_api_rc_rssi_in", "hw_api/rc_rssi"),
            ("~/hw_api_status_in", "hw_api/status"),
            ("~/safety_area_manager_diagnostics_in", "safety_area_manager/diagnostics"),
            ("~/mass_estimate_in", "control_manager/mass_estimate"),
            ("~/mass_nominal_in", "control_manager/mass_nominal"),
            ("~/mpc_tracker_diagnostics_in", "control_manager/mpc_tracker/estimation_diagnostics_info"),
            ("~/uav_status_in", "uav_status_acquisition/uav_status"),

            # Errorgraph topics
            ("~/errors_in", "errors"),
            ("~/errors_out", "root_errors"),
        ],
    )

    load_into_existing = LoadComposableNodes(
        target_container=container_name,
        composable_node_descriptions=[state_monitor_node],
        condition=UnlessCondition(standalone)
    )

    ld.add_action(load_into_existing)

    # #} end of state monitor node

    # #{ standalone container

    standalone_container = ComposableNodeContainer(
        namespace=robot_name,
        name=namespace + '_container',
        package='rclcpp_components',
        executable='component_container_mt',
        output="screen",
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        # prefix=['debug_roslaunch ' + os.ttyname(sys.stdout.fileno())],
        composable_node_descriptions=[state_monitor_node],
        parameters=[
            {'use_intra_process_comms': True},
            {'thread_num': os.cpu_count()},
            {'use_sim_time': use_sim_time},
        ],
        condition=IfCondition(standalone)
    )

    ld.add_action(standalone_container)

    # #} end of own container

    # #{ optional HTTP bridge for system health info

    system_health_http_enabled = LaunchConfiguration('system_health_http_enabled')
    ld.add_action(DeclareLaunchArgument(
        'system_health_http_enabled',
        default_value='false',
        description='If true, start an ASGI server exposing system_health_info over HTTP.',
    ))

    system_health_http_host = LaunchConfiguration('system_health_http_host')
    ld.add_action(DeclareLaunchArgument(
        'system_health_http_host',
        default_value='127.0.0.1',
        description='Host/IP used by the ASGI HTTP bridge.',
    ))

    system_health_http_port = LaunchConfiguration('system_health_http_port')
    ld.add_action(DeclareLaunchArgument(
        'system_health_http_port',
        default_value='8081',
        description='Port used by the ASGI HTTP bridge.',
    ))

    system_health_http_server = ExecuteProcess(
        cmd=[
            'python3',
            PathJoinSubstitution([this_pkg_path, 'scripts', 'system_health_http_server.py']),
            '--robot-name', robot_name,
            '--host', system_health_http_host,
            '--port', system_health_http_port,
        ],
        output='screen',
        condition=IfCondition(system_health_http_enabled),
    )
    ld.add_action(system_health_http_server)

    # #} end of optional HTTP bridge

    return ld
