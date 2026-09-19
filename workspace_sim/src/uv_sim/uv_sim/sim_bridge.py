"""Compatibility entry point; the implementation lives in ``uv_sim_bridge``."""

from uv_sim_bridge.sim_bridge import SimBridgeNode, main

__all__ = ['SimBridgeNode', 'main']

if __name__ == '__main__':
    main()
