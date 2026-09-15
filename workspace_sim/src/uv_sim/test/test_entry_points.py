"""Guard the console entry point that every SIL launch file invokes.

``ros2 run uv_sim sim_bridge`` executes the ``libexec`` wrapper, whose first
statement is ``from uv_sim.sim_bridge import main``.  If the module file is
truncated or half-saved, the whole simulation stack dies with a bare
``ImportError: cannot import name 'main'`` that points at the wrapper instead of
the real cause -- exactly what happened on 2026-09-15, when
``uv_sim/sim_bridge.py`` had been reduced to 0 bytes.

These checks keep that failure inside the test suite, with a message that names
the file and its size.
"""

from __future__ import annotations

from pathlib import Path

import uv_sim.sim_bridge as sim_bridge

# The module is ~780 lines / ~33 kB.  Anything far below that means the file was
# truncated, emptied or replaced by a stub.
MINIMUM_MODULE_BYTES = 20_000


def test_module_file_is_not_truncated() -> None:
    source = Path(sim_bridge.__file__)
    assert source.is_file(), f"module file missing: {source}"

    size = source.stat().st_size
    assert size >= MINIMUM_MODULE_BYTES, (
        f"{source} is only {size} bytes (expected >= {MINIMUM_MODULE_BYTES}); "
        "the file looks truncated -- restore it before launching the sim"
    )

    text = source.read_text(encoding="utf-8")
    for needle in ("class SimBridgeNode", "def main(", "def body_twist_from_odometry("):
        assert needle in text, f"{source} is missing {needle!r}"


def test_entry_point_is_importable() -> None:
    caller = getattr(sim_bridge, "main", None)
    assert callable(caller), (
        "uv_sim.sim_bridge.main is missing; "
        "'ros2 run uv_sim sim_bridge' would fail with ImportError"
    )


def test_sim_bridge_node_class_is_exported() -> None:
    assert hasattr(sim_bridge, "SimBridgeNode")
