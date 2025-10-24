import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/sanjay/ros2_workspaces/rcup_simul/src/rcup_simulations/install/rcup_simulations'
