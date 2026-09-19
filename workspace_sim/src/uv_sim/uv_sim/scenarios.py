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

# Public launch presets.  Competition presets select a world and reuse the
# ordinary simulator profile for bridge/perception parameters.  An explicit
# ``world:=`` always wins over the preset world.
PROFILE_ALIASES = {
    "sim_dev": ("sim_dev", None),
    "sim_ci": ("sim_ci", None),
    "guoshui": ("sim_dev", "guoshui_2026/cruise_seeded"),
    "guoshui_cruise": ("sim_dev", "guoshui_2026/cruise"),
    "guoshui_cruise_seeded": ("sim_dev", "guoshui_2026/cruise_seeded"),
    "sauvc_finals": ("sim_dev", "sauvc_2026/finals"),
    "sauvc_qualification": ("sim_dev", "sauvc_2026/qualification"),
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
