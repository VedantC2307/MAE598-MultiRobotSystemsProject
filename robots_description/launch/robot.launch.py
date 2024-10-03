# display_robot.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node 
import os

def generate_launch_description():
    package_dir = os.path.join(os.environ['HOME'], 'MultiRobotProject', 'src', 'robots_description')
    urdf_file = os.path.join(package_dir, 'urdf', 'trial_robot.urdf')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='both',
            parameters=[{'robot_description': robot_desc}]
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            parameters=[{'robot_description': robot_desc}],
        ),

        Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'),
    ])
