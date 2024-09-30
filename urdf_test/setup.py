from setuptools import setup
import os  # os 모듈 추가
from glob import glob

package_name = 'urdf_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # URDF 디렉토리 내 모든 파일 포함
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),

        # Launch 디렉토리 내 모든 파일 포함
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),

        # RViz 디렉토리 내 모든 파일 포함
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],    

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='poweq68@gmail.com',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rotate_wheel_node = urdf_test.rotate_wheel_node:main'
        ],
    },
)
