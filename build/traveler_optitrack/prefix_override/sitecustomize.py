import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/parnia/Projects/Turtle_workspace/install/traveler_optitrack'
