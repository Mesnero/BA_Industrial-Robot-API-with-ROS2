import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, RegisterEventHandler
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch.event_handlers import OnProcessExit

import xacro

import re
def remove_comments(text):
    pattern = r'<!--(.*?)-->'
    return re.sub(pattern, '', text, flags=re.DOTALL)

def generate_launch_description():
    robot_name_in_model = 'robco'
    package_name = 'robco_description'
    urdf_name = "demo_robot.xacro"

    pkg_share = FindPackageShare(package=package_name).find(package_name) 
    urdf_model_path = os.path.join(pkg_share, f'urdf/{urdf_name}')

    set_gazebo_model_database_uri = SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', '')
    set_gazebo_model_path = SetEnvironmentVariable('GAZEBO_MODEL_PATH', '/home/yo/robco_ws/src/robco_description:/usr/share/gazebo-11/models')

    # Start Gazebo server
    start_gazebo_cmd =  ExecuteProcess(
        cmd=['gazebo', '--verbose','-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen')


    # Because there is a line $(find mybot) in the urdf file, it needs to be compiled with xacro
    xacro_file = urdf_model_path
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    # params = {'robot_description': doc.toxml()}
    params = {'robot_description': remove_comments(doc.toxml())}

    # After starting the robot_state_publisher node, this node will publish the robot_description topic, 
    # the content of the topic is the content of the urdf model file, 
    # and it will subscribe to the /joint_states topic to get the joint data, 
    # and then publish the tf and tf_static topics.
    # Can the names of these nodes and topics be customized?
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': True}, params, {"publish_frequency":15.0}],
        output='screen'
    )

    # Launch the robot, generate the model in gazebo through the robot_description topic
    spawn_entity_cmd = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-entity', robot_name_in_model,  '-topic', 'robot_description'], output='screen')


    # # Launch the robot, this is to generate the model in gazebo by passing the file path. 
    # At this time, it is required that there is no xacro statement in the urdf file
    # spawn_entity_cmd = Node(
    #     package='gazebo_ros', 
    #     executable='spawn_entity.py',
    #     arguments=['-entity', robot_name_in_model,  '-file', urdf_model_path ], output='screen')
    
    # node_robot_state_publisher = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     arguments=[urdf_model_path],
    #     parameters=[{'use_sim_time': True}],
    #     output='screen'
    # )

    # When loading the urdf in gazebo, according to the settings of the urdf, will a joint_states node be started?
    # Joint state publisher
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    # Path execution controller, is that the action?
    # How does the system know that there is a my_group_controller controller?
    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'my_group_controller'],
        output='screen'
    )

    # Use the following two to control the startup sequence of each node
    # Listen to spawn_entity_cmd, when it exits (fully started), start load_joint_state_controller?
    close_evt1 =  RegisterEventHandler( 
            event_handler=OnProcessExit(
                target_action=spawn_entity_cmd,
                on_exit=[load_joint_state_controller],
            )
    )
    # Listen to load_joint_state_controller, when it exits (fully started), start load_joint_trajectory_controller?
    # How does moveit connect with the action provided by gazebo here?
    close_evt2 = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_joint_trajectory_controller],
            )
    )
    
    ld = LaunchDescription()

    ld.add_action(close_evt1)
    ld.add_action(close_evt2)

    ld.add_action(start_gazebo_cmd)
    ld.add_action(node_robot_state_publisher)
    ld.add_action(spawn_entity_cmd)

    return ld
