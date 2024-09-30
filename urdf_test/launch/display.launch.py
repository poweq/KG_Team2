import launch
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os 

def generate_launch_description():
    # Define launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    urdf = os.path.join(get_package_share_directory('urdf_test'), 'urdf', 'test_whill.urdf')

    with open(urdf, 'r') as read_urdf:
        robot_desc = read_urdf.read()

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
           'use_sim_time',
            default_value='false',),
        Node(
            package='joint_state_publisher',
            executable = 'joint_state_publisher',
            name = 'joint_state_publisher',
            output='screen',
                  parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_desc
            }]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            output='screen',
            arguments=['-d', os.path.join(get_package_share_directory('urdf_test'), 'rviz', 'urdf_rviz.rviz')] # Replace with your RViz config file path
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_desc
            }]
        )
    ])

if __name__ == '__main__':
    generate_launch_description()

