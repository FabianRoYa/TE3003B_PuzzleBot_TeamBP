import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/faroya/Desktop/ros2_ws/src/blackpearls_nav2_puzzlebot/install/blackpearls_nav2_puzzlebot'
