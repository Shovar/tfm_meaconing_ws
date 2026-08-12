import sys
if sys.prefix == '/Users/toni/robostack/.pixi/envs/jazzy':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/Users/toni/tfm_meaconing_ws/install/collaborative_detection'
