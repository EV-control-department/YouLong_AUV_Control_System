import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/origin/YouLong_AUV_Control_System/workspace_auv/install/uv_perception'
