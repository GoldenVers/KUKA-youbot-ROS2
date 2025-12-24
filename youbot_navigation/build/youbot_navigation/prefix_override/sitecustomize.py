import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/youssef/fyp_ws/src/youbot_navigation/install/youbot_navigation'
