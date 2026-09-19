### stonefish_ros2 package for ROS

This package delivers a ROS2 interface for the _Stonefish_ library. It also includes a standard simulator node, which loads the simulation scenario from a scenario description file (XML). The included parser extends the standard functionality of the _Stonefish_ library to enable search for files, resolution of parameters as well as a complete message interface. 

### Installation

1. Install the open-source [Stonefish](https://github.com/patrykcieslak/stonefish) library (*the same version as the ROS2 package!*).
2. Clone the *stonefish_ros2* package to your workspace.
3. Compile the workspace.

### Launching

To run the standard simulator node you have to include the 'stonefish_simulator.launch.py' file in your own launch file, overriding the default arguments.
Please refer to the documentation for details.

### Project-specific quick start (YouLong)

The public project entry point owns world and vehicle selection:

```bash
source /opt/ros/jazzy/setup.bash
source workspace_auv/install/setup.bash
source workspace_sim/install/setup.bash
ros2 launch uv_sim sim.launch.py \
  world:=guoshui_2026/cruise_seeded vehicle:=youlong
```

For a direct Stonefish parser invocation, pass the installed asset package as
`simulation_data` and a package-relative scenario:

```bash
ros2 launch stonefish_ros2 stonefish_simulator_nogpu.launch.py \
  scenario_desc:=worlds/guoshui_2026/guoshui_2026_cruise.scn \
  simulation_data:=$(ros2 pkg prefix uv_sim_assets)/share/uv_sim_assets
```

`uv_sim_bringup sim.launch.py scenario_desc:=...` remains available as the
legacy forwarding/compatibility entry point.

### Credits
This software was written and is continuously developed by Patryk Cieślak.

If you find this software useful in your research, please cite:

*Patryk Cieślak, "Stonefish: An Advanced Open-Source Simulation Tool Designed for Marine Robotics, With a ROS Interface", In Proceedings of MTS/IEEE OCEANS 2019, June 2019, Marseille, France*
```
@inproceedings{stonefish,
   author = {Cie{\'s}lak, Patryk},
   booktitle = {OCEANS 2019 - Marseille},
   title = {{Stonefish: An Advanced Open-Source Simulation Tool Designed for Marine Robotics, With a ROS Interface}},
   month = jun,
   year = {2019},
   doi={10.1109/OCEANSE.2019.8867434}}
```

### Support
I offer paid support on setting up the simulation of your own systems, including necessary 3D modelling (simplification of CAD models for physics, preparation of accurate visualisations, etc.), setup of simulation scenarios, development of new sensors, actuators, and custom features that do not require significant changes to the code base. Please contact me at [patryk.cieslak@udg.edu](mailto:patryk.cieslak@udg.edu).

### Funding
Currently there is no funding of this work. It is developed by the author following his needs and requests from other users.

### License
This is free software, published under the General Public License v3.0.
