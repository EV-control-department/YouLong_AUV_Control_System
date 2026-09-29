"""Stable world/vehicle names for the public ``uv_sim`` launch entry point."""

from __future__ import annotations

from pathlib import Path


WORLD_ALIASES = {
    "guoshui_2026/cruise": "worlds/guoshui_2026/guoshui_2026_cruise.scn",
    "guoshui_2026/cruise_seeded": (
        "worlds/guoshui_2026/guoshui_2026_cruise_seeded.scn"
    ),
    "sauvc_2026/finals": "worlds/sauvc_2026/sauvc_2026_finals.scn",
    "sauvc_2026/qualification": (
        "worlds/sauvc_2026/sauvc_2026_qualification.scn"
    ),
    "sauvc_2026/pool": "worlds/sauvc_2026/sauvc_pool.scn",
}


def resolve_world(assets_root: Path, value: str) -> Path:
    """Resolve a public world name or a package-relative ``.scn`` path."""

    candidate = WORLD_ALIASES.get(value, value)
    path = Path(candidate).expanduser()
    if not path.is_absolute():
        path = assets_root / path
    if not path.is_file():
        choices = ", ".join(sorted(WORLD_ALIASES))
        raise RuntimeError(
            f"unknown or missing world {value!r}; choose one of {choices}")
    return path.resolve()
