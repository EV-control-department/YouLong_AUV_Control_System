"""Guard the console entry points that every SIL launch file invokes.

``ros2 run uv_sim <name>`` imports ``<module>:<callable>`` from ``setup.py``.  If a
module file is truncated or half-saved, the whole simulation stack dies with a bare
``ImportError`` that points at the executable instead of the real cause -- exactly
what happened on 2026-09-15, when ``uv_sim/sim_bridge.py`` had been reduced to
0 bytes.

The checks are **derived from ``setup.py``** rather than listing node names, so
T5's split from one monolith into seven nodes (and any future node) is covered
without editing this file: whatever the package claims it can run must resolve,
be non-trivial and expose a callable entry.

The split-specific assertions (exactly seven node executables, no ``sim_bridge``,
no shadowing ``libexec`` wrapper) live in ``test_node_split.py``, because they are
about the shape of the split rather than about entry points resolving at all.
"""

from __future__ import annotations

import ast
import importlib
from pathlib import Path

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SETUP_PY = PACKAGE_ROOT / "setup.py"

#: A single node module is at least this large.  Only a truncated/emptied file
#: (the 2026-09-15 accident) can be smaller -- the guard is about that accident,
#: not about style.
MINIMUM_MODULE_BYTES = 1_000


def _console_scripts() -> dict[str, tuple[str, str]]:
    """``name -> (module, attribute)`` as declared in ``setup.py``."""
    tree = ast.parse(SETUP_PY.read_text(encoding="utf-8"))
    entries: dict[str, tuple[str, str]] = {}
    for node in ast.walk(tree):
        if not isinstance(node, ast.keyword) or node.arg != "entry_points":
            continue
        for value in ast.walk(node.value):
            if not isinstance(value, ast.Dict):
                continue
            keys = [key.value for key in value.keys if isinstance(key, ast.Constant)]
            if "console_scripts" not in keys:
                continue
            scripts = value.values[keys.index("console_scripts")]
            if not isinstance(scripts, ast.List):
                continue
            for item in scripts.elts:
                if isinstance(item, ast.Constant) and "=" in str(item.value):
                    name, _, target = str(item.value).partition("=")
                    module, _, attribute = target.strip().partition(":")
                    entries[name.strip()] = (module, attribute)
    return entries


@pytest.fixture(scope="module")
def scripts() -> dict[str, tuple[str, str]]:
    entries = _console_scripts()
    assert entries, f"no console_scripts parsed from {SETUP_PY}"
    return entries


def test_every_entry_point_module_is_present_and_not_truncated(scripts) -> None:
    for name, (module, _attribute) in sorted(scripts.items()):
        imported = importlib.import_module(module)
        source = Path(imported.__file__)
        assert source.is_file(), f"{name}: module file missing: {source}"

        size = source.stat().st_size
        assert size >= MINIMUM_MODULE_BYTES, (
            f"{source} is only {size} bytes (expected >= {MINIMUM_MODULE_BYTES}); "
            f"the file looks truncated -- restore it before launching the sim"
        )


def test_every_entry_point_is_importable(scripts) -> None:
    for name, (module, attribute) in sorted(scripts.items()):
        imported = importlib.import_module(module)
        entry = getattr(imported, attribute, None)
        assert callable(entry), (
            f"{module}:{attribute} (entry point {name!r}) is missing or not callable; "
            f"'ros2 run uv_sim {name}' would fail with ImportError"
        )
