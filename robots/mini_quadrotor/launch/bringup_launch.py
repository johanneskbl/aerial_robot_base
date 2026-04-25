#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression, TextSubstitution, Command, FindExecutable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


# ---------------------------------------------------------------------------
# Argument declarations  (name, default, description, (optional) choices)
# ---------------------------------------------------------------------------
_ARGS = [
    ("robot_model",         "mini_quadrotor",      "Name of the robot model ROS package"),
    ("robot_ns",            "mini_quadrotor",      "Namespace for all robot nodes"),
    ("real_machine",        "true",                "Use real machine specific bring-up inside model_launch", ["true", "false"]),
    ("main_rate",           "40.0",                "Core node main loop rate [Hz]"),
    ("estimation_mode",     "0",                   "Estimator mode on real machine: 0=egomotion, 1=experiment, 2=ground-truth", ["0", "1", "2"]),
    ("model_options",       "",                    "Extra xacro arguments passed verbatim to xacro"),
    ("headless",            "true",                "Run without GUI", ["true", "false"]),
    ("sim",                 "false",               "Launch Gazebo simulation", ["true", "false"]),
    ("sim_estimation_mode", "2",                   "Estimator mode in simulation: 0=egomotion, 1=experiment, 2=ground-truth", ["0", "1", "2"]),
    ("spawn_x",             "0.0",                 "Gazebo spawn X position [m] (sim only)"),
    ("spawn_y",             "0.0",                 "Gazebo spawn Y position [m] (sim only)"),
    ("spawn_z",             "0.5",                 "Gazebo spawn Z position [m] (sim only)"),
    ("robot_model_rviz",    "rviz_config.rviz",    "RViz config filename (resolved inside robot_model pkg/config/)"),
]


def sanity_check(context, *args, **kwargs):
    # Evaulate AFTER substitutions (e.g., sim and estimation_mode) are resolved, but BEFORE any nodes are launched
    # NOTE: Together with "choices" only real possibility to guardrail arguments
    real_machine = LaunchConfiguration("real_machine").perform(context)
    sim = LaunchConfiguration("sim").perform(context)
    if real_machine.lower() == "true" and sim.lower() == "true":
        raise RuntimeError("real_machine and sim arguments cannot both be true")

    rate = float(LaunchConfiguration("main_rate").perform(context))
    if rate <= 0 or rate > 200:
        raise RuntimeError(f"main_rate={rate} is not as expected. ")


def generate_launch_description():
    # ------------------------------------------------------------------
    # 1.  Declare CLI-overridable arguments
    # ------------------------------------------------------------------
    declared_args = [
        DeclareLaunchArgument(
            name,
            default_value=default_value,
            description=description,
            **({"choices": choices[0]} if choices else {})
        )
        for name, default_value, description, *choices in _ARGS
    ]

    # Resolve / Read at launch time (NOT AT IMPORT TIME)
    robot_model_pkg     = LaunchConfiguration("robot_model")
    robot_ns            = LaunchConfiguration("robot_ns")
    real_machine        = LaunchConfiguration("real_machine")
    main_rate           = LaunchConfiguration("main_rate")
    estimation_mode     = LaunchConfiguration("estimation_mode")
    model_options       = LaunchConfiguration("model_options")
    headless            = LaunchConfiguration("headless")
    sim                 = LaunchConfiguration("sim")
    sim_estimation_mode = LaunchConfiguration("sim_estimation_mode")
    spawn_x             = LaunchConfiguration("spawn_x")
    spawn_y             = LaunchConfiguration("spawn_y")
    spawn_z             = LaunchConfiguration("spawn_z")
    robot_model_rviz    = LaunchConfiguration("robot_model_rviz")

    # TODO: get plugin name from yaml file
    robot_model_plugin_name = {
        "robot_model_plugin_name": ParameterValue(
            "multirotor_robot_model",
            value_type=str
        )
    }

    active_estimation_mode = PythonExpression([
        "int('", sim_estimation_mode, "') if '", sim, "' == 'true' else int('", estimation_mode, "')"
    ])

    # ------------------------------------------------------------------
    # 2.  Derived paths
    # ------------------------------------------------------------------
    state_estimation_path = PathJoinSubstitution([
        FindPackageShare(robot_model_pkg),
        "config", "StateEstimation.yaml",
    ])

    servo_param_path = PathJoinSubstitution([
        FindPackageShare(robot_model_pkg),
        "config", "Servo.yaml",
    ])

    xacro_filename = PythonExpression([
        "'robot.gazebo.xacro' if '", sim, "' == 'true' else 'robot.urdf.xacro'"
    ])

    xacro_path = PathJoinSubstitution([
        FindPackageShare(robot_model_pkg),
        "urdf", xacro_filename,
    ])

    # Build the URDF/xacro string via the xacro CLI.
    # model_options is appended verbatim so callers can inject extra xacro args, e.g.,:
    #   model_options:="prop_num:=6 payload:=true"
    # and wrap in ParameterValue so ROS2 treats it as a string parameter
    robot_description = [
        FindExecutable(name="xacro"),
        TextSubstitution(text=" "),
        xacro_path,
        TextSubstitution(text=" "),
        model_options,
    ]
    robot_description_param = {
        "robot_description": ParameterValue(
            Command(robot_description),
            value_type=str
        )
    }

    sim_param_path = PathJoinSubstitution([
        FindPackageShare(robot_model_pkg),
        "config", "Simulation.yaml",
    ])

    # TODO: Use or remove
    rviz_config_path = PathJoinSubstitution([
        FindPackageShare(robot_model_pkg),
        "config", robot_model_rviz
    ])

    # ------------------------------------------------------------------
    # 3.  Nodes
    # ------------------------------------------------------------------
    core_node = Node(
        package="aerial_robot_core",
        executable="aerial_robot_core_node",
        name="aerial_robot_core",
        namespace=robot_ns,
        prefix=['gdb -ex run --args'],
        parameters=[
            {
                "main_rate": main_rate,
                "estimation.mode": active_estimation_mode,
                "use_sim_time": sim,
            },
            robot_description_param,
            robot_model_plugin_name,
            state_estimation_path,
        ],
        output="screen",
    )

    # ------------------------------------------------------------------
    # 4.  Call child launch files
    # ------------------------------------------------------------------
    # Robot model (URDF publisher, RViz, joint-state publisher …)
    model_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("aerial_robot_model"),
                "launch", "aerial_robot_model_launch.py",
            ])
        ),
        launch_arguments={
            "robot_model": robot_model_pkg,
            "robot_ns": robot_ns,
            "real_machine": real_machine,
            "model_options": model_options,
            "headless": headless,
            "robot_model_rviz": robot_model_rviz,
            "robot_description": robot_description,
            "sim": sim,
        }.items(),
    )

    # Gazebo simulation
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("aerial_robot_simulation"),
                "launch", "gazebo_launch.py",
            ])
        ),
        launch_arguments={
            "robot_ns": robot_ns,
            "headless": headless,
            "sim_param_path": sim_param_path,
            "spawn_x": spawn_x,
            "spawn_y": spawn_y,
            "spawn_z": spawn_z,
        }.items(),
        condition=IfCondition(sim),
    )

    # ------------------------------------------------------------------
    # 5.  Assemble LaunchDescription
    # ------------------------------------------------------------------
    ld = LaunchDescription()

    for arg in declared_args:
        ld.add_action(arg)

    ld.add_action(OpaqueFunction(function=sanity_check))
    ld.add_action(core_node)
    ld.add_action(model_launch)
    ld.add_action(sim_launch)

    return ld
