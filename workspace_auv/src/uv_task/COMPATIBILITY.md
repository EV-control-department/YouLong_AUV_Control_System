# Task compatibility baseline

The supported deployed baseline is ROS 2 Foxy with Python 3.8. Task code is
checked against both Ubuntu OpenCV 4.2 / NumPy 1.17 and the project environment.

From the repository root, after building the workspace:

```bash
source /opt/ros/foxy/setup.bash
source workspace_auv/install/setup.bash
python3 -B scripts/check_task_foxy.py
python3 -B scripts/check_task_foxy.py --system-opencv
```

These checks compile every task Python file, import every task module, verify
registered handlers and bundled YAML files, construct task helpers with mock
subscriptions, and exercise front-camera callbacks and ArUco decoding. They run
the entire task test suite without starting ROS nodes or issuing vehicle motion.

The checks must run before deploying task changes. A successful test under a
newer Python or ROS release does not replace the Foxy/Python 3.8 checks.

When editing tasks:

- Use Python 3.8 APIs. `str.removeprefix()` and `str.removesuffix()` require 3.9;
  validated camera prefixes can be removed with slicing.
- Keep each ROS logger call site at one severity. Use separate `if`/`else`
  calls for INFO and WARN; do not select logging methods through a ternary.
- Check API availability for OpenCV ArUco. Older releases use
  `DetectorParameters_create()`, `detectMarkers()` and `drawMarker()`.
- Keep postponed annotations when using newer type hint notation. Tests that
  extract methods through AST must preserve the annotations compiler flag.
- Keep BLINE speed parameters strictly between 0 and 0.18 m/s. The upper bound
  is exclusive; the task YAML validator rejects 0.18.

These are software compatibility checks. Camera calibration, perception
accuracy, actuator behavior and complete water trials require their own
deployment validation.
