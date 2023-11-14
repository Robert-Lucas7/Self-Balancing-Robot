import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable, ExecuteProcess)
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import xacro

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    urdf_file_name = 'urdf/two_wheeled_copied.xacro'

    print("urdf_file_name : {}".format(urdf_file_name))

    urdf = os.path.join(
        get_package_share_directory('sbr_pkg'),
        urdf_file_name)
    ld = LaunchDescription()
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')
    
    doc = xacro.process_file(urdf)
    robot_desc = doc.toprettyxml(indent='   ')
    
    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}]
        #arguments=[urdf]
        )
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}])
    
    execute_gazebo_cmd = ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so','-s', 'libgazebo_ros_init.so'], #Change gzserver to gazebo for gui
            output='screen')
    
    spawn_model_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='urdf_spawner',
        output='screen',
        arguments=[
            "-topic", "/robot_description", 
            "-entity", "cam_bot",
            "-x",'0',
            "-y", '0',
            "-z",'0.1'
        ])
    
    control_code_cmd = Node(
        package='sbr_pkg',
        executable='control',
        output='screen'
        )
    
    
    ld.add_action(declare_use_sim_time_cmd)
    
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(joint_state_publisher)
    ld.add_action(execute_gazebo_cmd)
    ld.add_action(spawn_model_cmd)
    ld.add_action(control_code_cmd)

    
    return ld