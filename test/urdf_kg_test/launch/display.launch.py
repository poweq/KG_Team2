import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory  # 패키지 경로 가져오기

def generate_launch_description():
    # urdf 파일 경로를 절대 경로로 설정
    urdf_file_path = os.path.join(
        get_package_share_directory('urdf_kg_test'), 'urdf', 'jdamr.urdf'
    )

    return LaunchDescription([
        # RViz 노드를 실행하며, URDF 파일을 로봇 설명으로 표시
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(get_package_share_directory('urdf_kg_test'), 'rviz', 'urdf_display.rviz')]
        ),
        # 로봇 상태 퍼블리셔 노드
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_file_path).read(), 'use_sim_time': False}]
        ),
        # 조인트 상태 퍼블리셔 노드
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'use_gui': False}]
        ),
    ])
