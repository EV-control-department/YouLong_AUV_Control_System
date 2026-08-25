cmake_minimum_required(VERSION 3.8)
project(zit6_control_core)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pybind11 REQUIRED)
# pybind11 的 pybind11NewTools 需要 Python3 Interpreter+Development 组件
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

# --- 确定 Python site-packages 安装路径(aament 惯例 lib/pythonX.Y/site-packages) ---
# colcon 会把各包 install/lib/python*.Y/site-packages 并入 PYTHONPATH。
if(DEFINED Python3_EXECUTABLE)
  execute_process(
    COMMAND "${Python3_EXECUTABLE}" -c
            "import sys; print(f'lib/python{sys.version_info.major}.{sys.version_info.minor}/site-packages')"
    OUTPUT_VARIABLE _py_lib
    OUTPUT_STRIP_TRAILING_WHITESPACE)
endif()
if(NOT _py_lib)
  set(_py_lib "lib/python3.12/site-packages")
endif()
set(PYTHON_INSTALL_DIR "${_py_lib}")
message(STATUS "zit6_control_core: PYTHON_INSTALL_DIR = ${PYTHON_INSTALL_DIR}")

# --- 原生控制核(固件纯数学 + verbatim 控制核软链 + 宿主单例桩) ---
# stubs/ 放在 include 路径最前,shadow 掉固件 FreeRTOS/MotionContext/SystemConfig,
# 使 verbatim CascadeController/SetpointRouter 宿主可编译。
add_library(zit6_core STATIC
  zit6_core/PID_Controller.cpp
  zit6_core/KinematicProfile.cpp
  zit6_core/CascadeController.cpp
  zit6_core/SetpointRouter.cpp
  zit6_core/motion_context_host.cpp
  zit6_core/ControllerHost.cpp
)
set_target_properties(zit6_core PROPERTIES
  POSITION_INDEPENDENT_CODE ON
)
target_include_directories(zit6_core PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/stubs
  ${CMAKE_CURRENT_SOURCE_DIR}/zit6_core
  ${EIGEN3_INCLUDE_DIR}
)

# --- pybind11 模块 ---
# 定位 pybind11 头目录(apt python3-pybind11 提供 /usr/include/pybind11)
find_path(PYBIND11_INCLUDE_DIR pybind11/pybind11.h)
if(NOT PYBIND11_INCLUDE_DIR)
  message(FATAL_ERROR "pybind11 headers not found (install: sudo apt install python3-pybind11)")
endif()
pybind11_add_module(zit6_control_core MODULE src/pybind_control_core.cpp)
target_link_libraries(zit6_control_core PRIVATE zit6_core)
target_include_directories(zit6_control_core PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/stubs
  ${CMAKE_CURRENT_SOURCE_DIR}/zit6_core
  ${PYBIND11_INCLUDE_DIR}
)

# 安装 Python 模块到 colcon 的 site-packages,使 import zit6_control_core 生效
install(TARGETS zit6_control_core
        DESTINATION ${PYTHON_INSTALL_DIR})

# --- 测试 ---
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_core test/test_core.py)
endif()

ament_package()
