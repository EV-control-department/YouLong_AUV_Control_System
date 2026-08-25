#include "ControllerHost.hpp"
#include "MotionContext.hpp"
#include "SystemConfig.hpp"

#include <pybind11/pybind11.h>

#include <array>

namespace py = pybind11;

namespace {

// 把一个 Python 序列(tuple/list/np.array)的 6 个元素转为 std::array<float,6>
// 用 C API PySequence_Fast,避免 pybind11 版本间 sequence/object 下标差异。
std::array<float, 6> to_arr6(const py::handle &h) {
  std::array<float, 6> out = {};
  PyObject *seq = PySequence_Fast(h.ptr(), "expected a sequence");
  if (!seq) {
    throw py::error_already_set();
  }
  const Py_ssize_t n = PySequence_Fast_GET_SIZE(seq);
  const Py_ssize_t m = n < 6 ? n : 6;
  for (Py_ssize_t i = 0; i < m; ++i) {
    PyObject *item = PySequence_Fast_GET_ITEM(seq, i);
    double v = PyFloat_AsDouble(item);
    if (v == -1.0 && PyErr_Occurred()) {
      Py_DECREF(seq);
      throw py::error_already_set();
    }
    out[i] = static_cast<float>(v);
  }
  Py_DECREF(seq);
  return out;
}

// 从 firmware config.json 的 chassis 段解析 ChassisConfig。
// 形如 {"planner_enabled": false, "x": {...}, "y": {...}, ... ,"yaw": {...}}
float get_axis_gain(py::dict axis, const char *key) {
  if (axis.contains(key)) {
    return static_cast<float>(axis[key].cast<double>());
  }
  return 0.0f;
}

auv::config::AxisConfig parse_axis(py::dict axis) {
  auv::config::AxisConfig a;
  a.pos_kp = get_axis_gain(axis, "pos_kp");
  a.pos_ki = get_axis_gain(axis, "pos_ki");
  a.pos_kd = get_axis_gain(axis, "pos_kd");
  a.pos_i_limit = get_axis_gain(axis, "pos_i_limit");
  a.pos_output_limit = get_axis_gain(axis, "pos_output_limit");
  a.vel_kp = get_axis_gain(axis, "vel_kp");
  a.vel_ki = get_axis_gain(axis, "vel_ki");
  a.vel_kd = get_axis_gain(axis, "vel_kd");
  a.vel_i_limit = get_axis_gain(axis, "vel_i_limit");
  a.vel_output_limit = get_axis_gain(axis, "vel_output_limit");
  a.max_v = get_axis_gain(axis, "max_v");
  a.max_a = get_axis_gain(axis, "max_a");
  a.mass = get_axis_gain(axis, "mass");
  a.drag = get_axis_gain(axis, "drag");
  return a;
}

auv::config::ChassisConfig parse_chassis(py::dict cfg) {
  auv::config::ChassisConfig c;
  if (cfg.contains("planner_enabled")) {
    c.planner_enabled = cfg["planner_enabled"].cast<bool>();
  }
  if (cfg.contains("x")) c.x = parse_axis(cfg["x"]);
  if (cfg.contains("y")) c.y = parse_axis(cfg["y"]);
  if (cfg.contains("z")) c.z = parse_axis(cfg["z"]);
  if (cfg.contains("roll")) c.roll = parse_axis(cfg["roll"]);
  if (cfg.contains("pitch")) c.pitch = parse_axis(cfg["pitch"]);
  if (cfg.contains("yaw")) c.yaw = parse_axis(cfg["yaw"]);
  return c;
}

} // namespace

PYBIND11_MODULE(zit6_control_core, m) {
  m.doc() = "ZIT6 native control core (host compile of the firmware cascade controller)";

  py::class_<auv::host::ControllerHost>(m, "Zit6Controller")
      // 从 firmware config.json 的 chassis 段字典构造
      .def(py::init([](py::dict config) {
        return new auv::host::ControllerHost(parse_chassis(config));
      }), py::arg("config"))

      .def("apply_config", [](auv::host::ControllerHost &c, py::dict config) {
        c.applyConfig(parse_chassis(config));
      }, py::arg("config"))

      .def("set_control_level", [](auv::host::ControllerHost &c, int level) {
        c.setControlLevel(static_cast<auv::motion::ControlLevel>(level));
      }, py::arg("level"))

      .def("update_setpoint",
           [](auv::host::ControllerHost &c, int control_key_level,
              py::handle val6, int mask, bool is_body, bool is_inc) {
             // 固件 MicroRosSubscriber::setpointCb 的 control_key&0x03 解码:
             //   0 -> POSITION, 1 -> VELOCITY, 2 -> ACTUATOR
             auv::motion::ControlLevel level;
             switch (control_key_level) {
               case 0: level = auv::motion::ControlLevel::POSITION; break;
               case 1: level = auv::motion::ControlLevel::VELOCITY; break;
               case 2: level = auv::motion::ControlLevel::ACTUATOR; break;
               default: level = auv::motion::ControlLevel::NONE; break;
             }
             auto v = to_arr6(val6);
             c.updateSetpoint(level, v.data(), static_cast<uint32_t>(mask), is_body, is_inc);
           },
           py::arg("control_key_level"), py::arg("val6"), py::arg("mask"),
           py::arg("is_body"), py::arg("is_inc"))

      .def("update_nav",
           [](auv::host::ControllerHost &c, py::handle pos_world6, py::handle vel_body6) {
             auv::motion::NavState n;
             n.pos_world = to_arr6(pos_world6);
             n.vel_body = to_arr6(vel_body6);
             c.updateNav(n);
           },
           py::arg("pos_world"), py::arg("vel_body"))

      .def("set_home_offset",
           [](auv::host::ControllerHost &c, py::handle pos6) {
             auto v = to_arr6(pos6);
             c.setHomeOffset(v.data());
           }, py::arg("pos_world"))

      .def("clear_home_offset", &auv::host::ControllerHost::clearHomeOffset)

      .def_property_readonly("has_home_offset",
                             &auv::host::ControllerHost::hasHomeOffset)

      .def("step", [](auv::host::ControllerHost &c) {
        auto f = c.step();
        py::list out;
        for (float x : f) out.append(x);
        return out;  // [Fx,Fy,Fz,Mroll,Mpitch,Myaw]
      })

      .def("configure_pid", [](auv::host::ControllerHost &c, int axis,
                               bool is_pos_ring, float kp, float ki, float kd,
                               float i_limit, float out_limit) {
        c.configurePID(axis, is_pos_ring, kp, ki, kd, i_limit, out_limit);
      }, py::arg("axis"), py::arg("is_pos_ring"), py::arg("kp"), py::arg("ki"),
         py::arg("kd"), py::arg("i_limit"), py::arg("out_limit"))

      .def_property_readonly("control_level", [](const auv::host::ControllerHost &c) {
        return static_cast<int>(c.getControlLevel());
      });
}
