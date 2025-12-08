import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/haqi/fp-robotik/flappy-bird-bot-3D/install/drone_collision'
