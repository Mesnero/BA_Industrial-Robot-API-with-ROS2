from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

# Constants
PACKAGE_NAME = 'robco_description'
WORLD_FILE_NAME = 'my_world.sdf'
URDF_FILE_NAME = 'demo_robot.xacro'
ROBOT_NAME = 'demo_robot'
GAZEBO_ROS_PACKAGE_NAME = 'gazebo_ros'
GAZEBO_LAUNCH_FILE_NAME = 'gazebo.launch.py'

def generate_launch_description():
    # Find the package path
    package_path = os.popen(f'ros2 pkg prefix {PACKAGE_NAME}').read().strip()

    # Full path to the world file
    world_path = os.path.join(package_path, 'share', PACKAGE_NAME, 'worlds', WORLD_FILE_NAME)

    # Full path to the URDF file
    urdf_path = os.path.join(package_path, 'share', PACKAGE_NAME, 'urdf', URDF_FILE_NAME)

    # Find the gazebo_ros package path
    gazebo_ros_path = os.popen(f'ros2 pkg prefix {GAZEBO_ROS_PACKAGE_NAME}').read().strip()

    # Full path to the gazebo launch file
    gazebo_launch_path = os.path.join(gazebo_ros_path, 'share', GAZEBO_ROS_PACKAGE_NAME, 'launch', GAZEBO_LAUNCH_FILE_NAME)

    return LaunchDescription([
        # Launch gazebo with the world file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([gazebo_launch_path]),
            launch_arguments={'world': world_path}.items()
        ),
        # Launches Node to spawn the robot in gazebo
        Node(
            package=GAZEBO_ROS_PACKAGE_NAME,
            executable='spawn_entity.py',
            arguments=['-entity', ROBOT_NAME, '-file', urdf_path],
            output='screen'
        )
    ])

