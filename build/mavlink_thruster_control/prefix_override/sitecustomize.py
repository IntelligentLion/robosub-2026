import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/robosub/UPDATEDCODE/install/mavlink_thruster_control'
