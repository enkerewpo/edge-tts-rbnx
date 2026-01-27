import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/syswonder/.robonix/packages/edge-tts-rbnx/install/edge_tts_rbnx'
