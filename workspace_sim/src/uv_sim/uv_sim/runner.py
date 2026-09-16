"""Shared node runner: one ``main()`` body for every ``uv_sim`` entry point.

``ros2 run`` and ``ros2 launch`` both expect the entry point to return cleanly
when the user hits Ctrl-C.  Before T5 this lived (without the interrupt handling)
inside ``sim_bridge.main``; the seven nodes T5 introduces would otherwise each
carry a copy of this boilerplate.

The teardown order matters: ``check_dvl_frame.py`` documents that destroying a
node while an executor is still spinning aborts the process (``terminate called
without an active exception``, exit 134), and this runner is the single place
that has to get it right for the simulation nodes.
"""

from __future__ import annotations

from typing import Callable

import rclpy
from rclpy.node import Node


def run(node_factory: Callable[[], Node], args=None) -> int:
    """Instantiate ``node_factory()``, spin it, and shut down cleanly.

    Returns 0 in every expected path (including Ctrl-C); a constructor failure
    propagates so ``ros2 launch`` shows the real traceback instead of a silent
    exit-0.
    """
    rclpy.init(args=args)
    node = None
    try:
        node = node_factory()
        rclpy.spin(node)
    except KeyboardInterrupt:
        rclpy.logging.get_logger('uv_sim').info(
            f"{type(node).__name__ if node else 'node'} interrupted, shutting down"
        )
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0
