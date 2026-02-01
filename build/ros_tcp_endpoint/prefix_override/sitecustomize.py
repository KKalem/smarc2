import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/smarc2user/colcon_ws/src/smarc2/install/ros_tcp_endpoint'
