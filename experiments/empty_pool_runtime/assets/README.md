# uv_sim_assets

This package is the source of truth for Stonefish data used by YouLong.

```text
vehicles/youlong/       canonical vehicle description and visual/physical meshes
worlds/                 maintained competition environments
objects/                reusable environment geometry
textures/               reusable textures
tools/                  deterministic generators and validators
legacy_data/            read-only migration fixtures for old scenario_desc names
```

The canonical vehicle is `vehicles/youlong/model/youlong.scn`. Its physical
mesh is used by Stonefish for collision and submerged hydrodynamics; visual
meshes are kept separately for rendering. `vehicle_baseline.yaml` records the
topology repair and the uncalibrated baseline status.

Run the structural gate with:

```bash
python3 tools/validate_assets.py .
# CMake/CTest also runs the deterministic vehicle and world generator check.
ctest --test-dir build/uv_sim_assets --output-on-failure
```

When a Stonefish no-GPU executable is available, add
`--stonefish-binary /path/to/stonefish_simulator_nogpu`. The runtime gate
stages a temporary copy and removes only camera sensors because Stonefish's
console application has no rendering context; the checked-in vehicle and its
camera topics remain unchanged.

| Old scenario name | Compatibility location | New public world name |
| --- | --- | --- |
| `guoshui_2026_cruise.scn` | canonical package path | `guoshui_2026/cruise` |
| `guoshui_2026_cruise_seeded.scn` | canonical package path | `guoshui_2026/cruise_seeded` |
| `sauvc_2026_finals.scn` | canonical package path | `sauvc_2026/finals` |
| `sauvc_2026_finals_with girona.scn` | canonical package path | `sauvc_2026/finals` |
| `sauvc_2026_qualification.scn` | canonical package path | `sauvc_2026/qualification` |
| `sauvc_pool.scn` | canonical package path | `sauvc_2026/pool` |
| `console_test.scn`, `simple.scn` | `worlds/examples/` | examples |
| `girona500auv_*.scn` | `worlds/examples/` | examples |
| `underwater_xunyun.scn` | `legacy_data/` | use `scenario_desc:=` during migration |

All 118 files formerly under `stonefish_ros2/Data` are retained under
`legacy_data/` with the same basename. Maintained scenarios and the Girona
fixtures additionally have canonical copies under `worlds/`:

| Legacy family | New location |
| --- | --- |
| `guoshui_2026_*` | `worlds/guoshui_2026/` with `objects/guoshui_2026/` and `textures/common/` |
| `sauvc_2026_*` | `worlds/sauvc_2026/` |
| `underwater_*`, `wuurc_*` | `worlds/legacy/` (compatibility fixtures) |
| `girona500auv_*`, `console_test.scn`, `simple.scn` | `worlds/examples/` |
| `xunyun_fixed.scn` | `vehicles/youlong/model/youlong.scn` (canonical) |
| `xunyun.scn` and remaining meshes | `legacy_data/` (not part of the YouLong baseline) |
