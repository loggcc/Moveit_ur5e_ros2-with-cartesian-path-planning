import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ariel/ur5e_ws/src/install/controller_pkg_demo'
