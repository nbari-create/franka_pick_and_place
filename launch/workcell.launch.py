from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except (FileNotFoundError, yaml.YAMLError):
        return None


def generate_launch_description():

    franka_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('franka_gazebo_bringup'),
                'launch', 'moveit_gazebo_franka_arm_example_controller.launch.py'
            )
        ),
        launch_arguments={
            'robot_type':    'fr3',
            'load_gripper':  'true',
            'franka_hand':   'franka_hand',
            'controller':    'joint_trajectory_controller',
            'rviz':          'true',
        }.items()
    )

    franka_xacro = os.path.join(
        get_package_share_directory('franka_gazebo_bringup'),
        'urdf', 'franka_arm.gazebo.xacro'
    )
    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', franka_xacro,
                 ' robot_type:=fr3', ' hand:=true',
                 ' ros2_control:=true', ' gazebo:=true',
                 ' ee_id:=franka_hand', ' xyz:="0 0 0.0"']),
        value_type=str
    )

    franka_srdf_xacro = os.path.join(
        get_package_share_directory('franka_description'),
        'robots', 'fr3', 'fr3.srdf.xacro'
    )
    robot_description_semantic = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', franka_srdf_xacro,
                 ' hand:=true', ' ee_id:=franka_hand']),
        value_type=str
    )

    kinematics_yaml = load_yaml(
        'franka_fr3_moveit_config', 'config/kinematics.yaml')

    ompl_planning_yaml = load_yaml(
        'franka_fr3_moveit_config', 'config/ompl_planning.yaml')

    planner_configs_list = [
        'SBLkConfigDefault', 'ESTkConfigDefault', 'LBKPIECEkConfigDefault',
        'BKPIECEkConfigDefault', 'KPIECEkConfigDefault', 'RRTkConfigDefault',
        'RRTConnectkConfigDefault', 'RRTstarkConfigDefault', 'TRRTkConfigDefault',
        'PRMkConfigDefault', 'PRMstarkConfigDefault', 'FMTkConfigDefault',
        'BFMTkConfigDefault', 'PDSTkConfigDefault', 'STRIDEkConfigDefault',
        'BiTRRTkConfigDefault', 'LBTRRTkConfigDefault', 'BiESTkConfigDefault',
        'ProjESTkConfigDefault', 'LazyPRMkConfigDefault', 'LazyPRMstarkConfigDefault',
        'SPARSkConfigDefault', 'SPARStwokConfigDefault', 'TrajOptDefault',
    ]

    # ── Dice a MoveItCpp QUALI pipeline caricare e in che namespace cercarle ──
    planning_pipelines_options = {
        'planning_pipelines': {
            'pipeline_names': ['move_group'],
            'namespace': 'move_group',
        }
    }

    planning_pipeline_config = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization '
                                'default_planner_request_adapters/ResolveConstraintFrames '
                                'default_planner_request_adapters/FixWorkspaceBounds '
                                'default_planner_request_adapters/FixStartStateBounds '
                                'default_planner_request_adapters/FixStartStateCollision '
                                'default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error': 0.1,
        }
    }
    if ompl_planning_yaml:
        planning_pipeline_config['move_group'].update(ompl_planning_yaml)
        planning_pipeline_config['move_group']['fr3_arm'] = {'planner_configs': planner_configs_list}
        planning_pipeline_config['move_group']['fr3_hand'] = {'planner_configs': planner_configs_list}

    moveit_simple_controllers_yaml = load_yaml(
        'franka_fr3_moveit_config', 'config/fr3_controllers_gazebo.yaml') or {}
    moveit_controllers = {
        'moveit_simple_controller_manager': moveit_simple_controllers_yaml,
        'moveit_controller_manager': 'moveit_simple_controller_manager'
                                     '/MoveItSimpleControllerManager',
    }

    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    main_node = TimerAction(
        period=25.0,
        actions=[
            Node(
                package='myworkcell_core',
                executable='main_node',
                output='screen',
                parameters=[
                    {'robot_description': robot_description,
                     'robot_description_semantic': robot_description_semantic,
                     'use_sim_time': True},
                    kinematics_yaml,
                    planning_pipelines_options,
                    planning_pipeline_config,
                    trajectory_execution,
                    moveit_controllers,
                    planning_scene_monitor_parameters,
                ],
            )
        ]
    )

    return LaunchDescription([franka_gazebo, main_node])
