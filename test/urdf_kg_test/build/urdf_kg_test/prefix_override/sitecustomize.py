import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/pi/ros2_ws/src/urdf_kg_test/install/urdf_kg_test'
