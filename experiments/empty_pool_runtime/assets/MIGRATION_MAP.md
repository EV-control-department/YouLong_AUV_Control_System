# Stonefish asset migration map

The complete source inventory is the 118-file `legacy_data/` archive. Every row retains the original file at `legacy_data/<basename>` for compatibility; canonical destinations below are the maintained copies used by `uv_sim_assets`.

| Original `stonefish_ros2/Data` file | Maintained destination(s) |
| --- | --- |
| `OIP.png` | `legacy_data/OIP.png` |
| `OIP_floor_tiled.png` | `legacy_data/OIP_floor_tiled.png` |
| `OIP_tiled.png` | `legacy_data/OIP_tiled.png` |
| `OIP_wall_tiled.png` | `legacy_data/OIP_wall_tiled.png` |
| `aquadelmo.obj` | `legacy_data/aquadelmo.obj` |
| `aquadelmo_phy.obj` | `legacy_data/aquadelmo_phy.obj` |
| `aquadelmo_tex.png` | `legacy_data/aquadelmo_tex.png` |
| `arrow_sign_east.png` | `legacy_data/arrow_sign_east.png` |
| `arrow_sign_west.png` | `legacy_data/arrow_sign_west.png` |
| `aruco_4x4_id1.png` | `legacy_data/aruco_4x4_id1.png` |
| `aruco_4x4_id2.png` | `legacy_data/aruco_4x4_id2.png` |
| `aruco_4x4_id3.png` | `legacy_data/aruco_4x4_id3.png` |
| `aruco_4x4_id4.png` | `legacy_data/aruco_4x4_id4.png` |
| `aruco_4x4_id5.png` | `legacy_data/aruco_4x4_id5.png` |
| `aruco_4x4_id6.png` | `legacy_data/aruco_4x4_id6.png` |
| `aruco_marker_uv.obj` | `legacy_data/aruco_marker_uv.obj` |
| `aruco_plate_base.png` | `legacy_data/aruco_plate_base.png` |
| `base_link_hydro.obj` | `legacy_data/base_link_hydro.obj` |
| `base_link_uji_hydro.obj` | `legacy_data/base_link_uji_hydro.obj` |
| `battery_pod.obj` | `legacy_data/battery_pod.obj`, `vehicles/youlong/meshes/visual/battery_pod.obj` |
| `boat.obj` | `legacy_data/boat.obj` |
| `boat_gra.obj` | `legacy_data/boat_gra.obj` |
| `canyon.obj` | `legacy_data/canyon.obj` |
| `canyon.png` | `legacy_data/canyon.png` |
| `checker.png` | `legacy_data/checker.png` |
| `cirs_tank.obj` | `legacy_data/cirs_tank.obj` |
| `cone.obj` | `legacy_data/cone.obj` |
| `console_test.scn` | `legacy_data/console_test.scn`, `worlds/examples/console_test.scn` |
| `cylinder_small.obj` | `legacy_data/cylinder_small.obj` |
| `cylinder_tilt.stl` | `legacy_data/cylinder_tilt.stl` |
| `dragon.obj` | `legacy_data/dragon.obj` |
| `duct.obj` | `legacy_data/duct.obj`, `vehicles/youlong/meshes/visual/duct.obj` |
| `duct_hydro.obj` | `legacy_data/duct_hydro.obj`, `vehicles/youlong/meshes/physical/duct_hydro.obj` |
| `duct_tex.png` | `legacy_data/duct_tex.png`, `vehicles/youlong/meshes/visual/duct_tex.png` |
| `eeprobe_hydro.obj` | `legacy_data/eeprobe_hydro.obj` |
| `fingerA_hydro.obj` | `legacy_data/fingerA_hydro.obj` |
| `fix_cams.py` | `legacy_data/fix_cams.py` |
| `fix_sensors.py` | `legacy_data/fix_sensors.py` |
| `funnel.obj` | `legacy_data/funnel.obj` |
| `gainfit.oct` | `legacy_data/gainfit.oct` |
| `generate_guoshui_2026_scene.py` | `legacy_data/generate_guoshui_2026_scene.py`, `tools/generate_guoshui_2026_scene.py` |
| `generate_guoshui_gate_parts.py` | `legacy_data/generate_guoshui_gate_parts.py`, `tools/generate_guoshui_gate_parts.py` |
| `girona500auv_console.scn` | `legacy_data/girona500auv_console.scn`, `worlds/examples/girona500auv_console.scn` |
| `girona500auv_full copy.scn` | `legacy_data/girona500auv_full copy.scn`, `worlds/examples/girona500auv_full copy.scn` |
| `girona500auv_full.scn` | `legacy_data/girona500auv_full.scn`, `worlds/examples/girona500auv_full.scn` |
| `guoshui_2026_collection_collision_plane.obj` | `legacy_data/guoshui_2026_collection_collision_plane.obj`, `objects/guoshui_2026/guoshui_2026_collection_collision_plane.obj` |
| `guoshui_2026_collection_net.obj` | `legacy_data/guoshui_2026_collection_net.obj`, `objects/guoshui_2026/guoshui_2026_collection_net.obj` |
| `guoshui_2026_collection_net_bottom_diagonal.obj` | `legacy_data/guoshui_2026_collection_net_bottom_diagonal.obj`, `objects/guoshui_2026/guoshui_2026_collection_net_bottom_diagonal.obj` |
| `guoshui_2026_collection_net_diagonal.obj` | `legacy_data/guoshui_2026_collection_net_diagonal.obj`, `objects/guoshui_2026/guoshui_2026_collection_net_diagonal.obj` |
| `guoshui_2026_collection_net_side_diagonal.obj` | `legacy_data/guoshui_2026_collection_net_side_diagonal.obj`, `objects/guoshui_2026/guoshui_2026_collection_net_side_diagonal.obj` |
| `guoshui_2026_cruise.scn` | `legacy_data/guoshui_2026_cruise.scn`, `worlds/guoshui_2026/guoshui_2026_cruise.scn` |
| `guoshui_2026_cruise_seeded.scn` | `legacy_data/guoshui_2026_cruise_seeded.scn`, `worlds/guoshui_2026/guoshui_2026_cruise_seeded.scn` |
| `guoshui_2026_gate_red_pipes.obj` | `legacy_data/guoshui_2026_gate_red_pipes.obj`, `objects/guoshui_2026/guoshui_2026_gate_red_pipes.obj` |
| `guoshui_2026_gate_red_sleeves.obj` | `legacy_data/guoshui_2026_gate_red_sleeves.obj`, `objects/guoshui_2026/guoshui_2026_gate_red_sleeves.obj` |
| `guoshui_2026_gate_white_sleeves.obj` | `legacy_data/guoshui_2026_gate_white_sleeves.obj`, `objects/guoshui_2026/guoshui_2026_gate_white_sleeves.obj` |
| `guoshui_2026_gate_white_supports.obj` | `legacy_data/guoshui_2026_gate_white_supports.obj`, `objects/guoshui_2026/guoshui_2026_gate_white_supports.obj` |
| `guoshui_2026_gates.obj` | `legacy_data/guoshui_2026_gates.obj`, `objects/guoshui_2026/guoshui_2026_gates.obj` |
| `guoshui_2026_pool_floor_tiled.png` | `legacy_data/guoshui_2026_pool_floor_tiled.png`, `textures/common/guoshui_2026_pool_floor_tiled.png` |
| `guoshui_2026_pool_wall_long_tiled.png` | `legacy_data/guoshui_2026_pool_wall_long_tiled.png`, `textures/common/guoshui_2026_pool_wall_long_tiled.png` |
| `guoshui_2026_pool_wall_side_tiled.png` | `legacy_data/guoshui_2026_pool_wall_side_tiled.png`, `textures/common/guoshui_2026_pool_wall_side_tiled.png` |
| `guoshui_2026_pvc_sleeve.obj` | `legacy_data/guoshui_2026_pvc_sleeve.obj`, `objects/guoshui_2026/guoshui_2026_pvc_sleeve.obj` |
| `guoshui_2026_red_ring.obj` | `legacy_data/guoshui_2026_red_ring.obj`, `objects/guoshui_2026/guoshui_2026_red_ring.obj` |
| `guoshui_2026_seeded_gate_red_pipes.obj` | `legacy_data/guoshui_2026_seeded_gate_red_pipes.obj`, `objects/guoshui_2026/guoshui_2026_seeded_gate_red_pipes.obj` |
| `guoshui_2026_seeded_gate_red_sleeves.obj` | `legacy_data/guoshui_2026_seeded_gate_red_sleeves.obj`, `objects/guoshui_2026/guoshui_2026_seeded_gate_red_sleeves.obj` |
| `guoshui_2026_seeded_gate_white_sleeves.obj` | `legacy_data/guoshui_2026_seeded_gate_white_sleeves.obj`, `objects/guoshui_2026/guoshui_2026_seeded_gate_white_sleeves.obj` |
| `guoshui_2026_seeded_gate_white_supports.obj` | `legacy_data/guoshui_2026_seeded_gate_white_supports.obj`, `objects/guoshui_2026/guoshui_2026_seeded_gate_white_supports.obj` |
| `hinge.obj` | `legacy_data/hinge.obj` |
| `hull_hydro.obj` | `legacy_data/hull_hydro.obj` |
| `hull_hydro2.obj` | `legacy_data/hull_hydro2.obj` |
| `human.obj` | `legacy_data/human.obj` |
| `icosphere.obj` | `legacy_data/icosphere.obj` |
| `link1_hydro.obj` | `legacy_data/link1_hydro.obj` |
| `link2_hydro.obj` | `legacy_data/link2_hydro.obj` |
| `link3_hydro.obj` | `legacy_data/link3_hydro.obj` |
| `link4_hydro.obj` | `legacy_data/link4_hydro.obj` |
| `link4_tex.png` | `legacy_data/link4_tex.png` |
| `link4ft_hydro.obj` | `legacy_data/link4ft_hydro.obj` |
| `multi_object_test.obj` | `legacy_data/multi_object_test.obj` |
| `pool_wall.png` | `legacy_data/pool_wall.png` |
| `propeller.obj` | `legacy_data/propeller.obj`, `vehicles/youlong/meshes/visual/propeller.obj` |
| `propeller_air.obj` | `legacy_data/propeller_air.obj`, `vehicles/youlong/meshes/visual/propeller_air.obj` |
| `propeller_tex.png` | `legacy_data/propeller_tex.png`, `vehicles/youlong/meshes/visual/propeller_tex.png` |
| `rope_color.jpg` | `legacy_data/rope_color.jpg` |
| `rope_normal.png` | `legacy_data/rope_normal.png` |
| `sand_normal.png` | `legacy_data/sand_normal.png` |
| `sauvc_2026_finals.scn` | `legacy_data/sauvc_2026_finals.scn`, `worlds/sauvc_2026/sauvc_2026_finals.scn` |
| `sauvc_2026_finals_with girona.scn` | `legacy_data/sauvc_2026_finals_with girona.scn` |
| `sauvc_2026_qualification.scn` | `legacy_data/sauvc_2026_qualification.scn`, `worlds/sauvc_2026/sauvc_2026_qualification.scn` |
| `sauvc_pool.scn` | `legacy_data/sauvc_pool.scn`, `worlds/sauvc_2026/sauvc_pool.scn` |
| `simple.scn` | `legacy_data/simple.scn`, `worlds/examples/simple.scn` |
| `sphere_R=1.obj` | `legacy_data/sphere_R=1.obj` |
| `tank.obj` | `legacy_data/tank.obj` |
| `terrain.obj` | `legacy_data/terrain.obj` |
| `terrain.png` | `legacy_data/terrain.png` |
| `terrain16b.png` | `legacy_data/terrain16b.png` |
| `terrain_small.png` | `legacy_data/terrain_small.png` |
| `test.oct` | `legacy_data/test.oct` |
| `torus_R=1_r=025.obj` | `legacy_data/torus_R=1_r=025.obj` |
| `underwater_test.scn` | `legacy_data/underwater_test.scn`, `worlds/legacy/underwater_test.scn` |
| `underwater_xunyun.scn` | `legacy_data/underwater_xunyun.scn`, `worlds/legacy/underwater_xunyun.scn` |
| `uv_grid_cube_tex.png` | `legacy_data/uv_grid_cube_tex.png` |
| `uv_grid_tex.png` | `legacy_data/uv_grid_tex.png` |
| `vbar_hydro.obj` | `legacy_data/vbar_hydro.obj` |
| `vbs_max.obj` | `legacy_data/vbs_max.obj` |
| `vbs_min.obj` | `legacy_data/vbs_min.obj` |
| `wuurc_murc_2026_auv.scn` | `legacy_data/wuurc_murc_2026_auv.scn`, `worlds/legacy/wuurc_murc_2026_auv.scn` |
| `xunyun.scn` | `legacy_data/xunyun.scn` |
| `xunyun_collision_hydro.obj` | `legacy_data/xunyun_collision_hydro.obj`, `vehicles/youlong/meshes/physical/youlong_collision_hydro.obj` |
| `xunyun_fixed.scn` | `legacy_data/xunyun_fixed.scn`, `vehicles/youlong/model/youlong.scn` |
| `xy_aft_bracket.obj` | `legacy_data/xy_aft_bracket.obj`, `vehicles/youlong/meshes/visual/xy_aft_bracket.obj` |
| `xy_buoyancy_block.obj` | `legacy_data/xy_buoyancy_block.obj`, `vehicles/youlong/meshes/visual/xy_buoyancy_block.obj` |
| `xy_core_enclosure.obj` | `legacy_data/xy_core_enclosure.obj`, `vehicles/youlong/meshes/visual/xy_core_enclosure.obj` |
| `xy_forward_bracket.obj` | `legacy_data/xy_forward_bracket.obj`, `vehicles/youlong/meshes/visual/xy_forward_bracket.obj` |
| `xy_ins_base_plate.obj` | `legacy_data/xy_ins_base_plate.obj`, `vehicles/youlong/meshes/visual/xy_ins_base_plate.obj` |
| `xy_left_sideplate.obj` | `legacy_data/xy_left_sideplate.obj`, `vehicles/youlong/meshes/visual/xy_left_sideplate.obj` |
| `xy_right_sideplate.obj` | `legacy_data/xy_right_sideplate.obj`, `vehicles/youlong/meshes/visual/xy_right_sideplate.obj` |
| `xy_top_plate.obj` | `legacy_data/xy_top_plate.obj`, `vehicles/youlong/meshes/visual/xy_top_plate.obj` |
| `yellow_triangle.png` | `legacy_data/yellow_triangle.png` |
