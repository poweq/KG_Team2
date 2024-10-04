from setuptools import setup
import os
from glob import glob

package_name = 'urdf_kg_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch 파일 설치
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        
        # URDF 파일 설치
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        
        # RViz 설정 파일 설치
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='poweq68@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'teleop_wheel = urdf_kg_test.teleop_wheel:main',
        ],
    },
)
