import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/useradd/ros2_ws/src/install/robot_initial_pose'
