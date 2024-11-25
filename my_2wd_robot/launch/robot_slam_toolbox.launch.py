# import os
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
# from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch_ros.actions import Node
# from launch_ros.substitutions import FindPackageShare
# from launch_ros.actions import ComposableNodeContainer
# from launch_ros.descriptions import ComposableNode
# from ament_index_python import get_package_share_directory


# def generate_launch_description():
#     use_sim_time = True

#     description_launch_path = PathJoinSubstitution(
#         [FindPackageShare('my_2wd_robot'), 'launch', 'robot_gazebo.launch.py']
#     )

#     slam_params_file = os.path.join(
#         get_package_share_directory('my_2wd_robot'),
#         'config',
#         'mapper_params_online_async.yaml'
#     )


#     return LaunchDescription([
 

#        Node(
#            package='slam_toolbox',
#            executable='online_async_launch.py',
#            name='slam_toolbox',
#            output='screen',
#            parameters=[{'slam_params_file':slam_params_file}, {'use_sim_time': use_sim_time}]
#        )

       

#         IncludeLaunchDescription(
#             PythonLaunchDescriptionSource(description_launch_path),
#             launch_arguments={
#                 'use_sim_time': str(use_sim_time),
#                 'publish_joints': 'true',
#             }.items()
#         )
#     ])



import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Path to the robot description launch file
    description_launch_path = PathJoinSubstitution(
        [FindPackageShare('my_2wd_robot'), 'launch', 'robot_gazebo.launch.py']
    )

    # Path to the SLAM parameters file
    slam_params_file = os.path.join(
        get_package_share_directory('my_2wd_robot'),
        'config',
        'mapper_params_online_async.yaml'
    )

    # Return the launch description
    return LaunchDescription([
        # Declare use_sim_time argument
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time (true for Gazebo, false for real robot)'
        ),

        # Launch the SLAM Toolbox node
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'slam_params_file': slam_params_file}]
        ),

        # Include the robot Gazebo launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch_path),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'publish_joints': 'true',
            }.items()
        )
    ])
