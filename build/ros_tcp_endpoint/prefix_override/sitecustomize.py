import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/zephyrr1/Upper_Limb_Rehabilitation_Robot_ws/install/ros_tcp_endpoint'
