cmake_minimum_required(VERSION 3.8)
project(uv_sim_assets VERSION 0.1.0)

find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

# Assets are data, not C++ sources.  Keeping the complete tree under one share
# directory gives Stonefish a stable simulation_data root for relative paths
# and root-level scenario includes.
install(DIRECTORY
  vehicles
  worlds
  objects
  textures
  config
  legacy_data
  DESTINATION share/${PROJECT_NAME}
)
install(FILES README.md MIGRATION_MAP.md DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  tools/generate_vehicle.py
  tools/generate_guoshui_2026_scene.py
  tools/generate_guoshui_gate_parts.py
  tools/check_reproducibility.py
  tools/validate_assets.py
  DESTINATION share/${PROJECT_NAME}/tools
)

if(BUILD_TESTING)
  include(CTest)
  add_test(
    NAME uv_sim_assets_validate
    COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_CURRENT_SOURCE_DIR}/tools/validate_assets.py
            ${CMAKE_CURRENT_SOURCE_DIR}
  )
  add_test(
    NAME uv_sim_assets_reproducible
    COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_CURRENT_SOURCE_DIR}/tools/check_reproducibility.py
  )
  # Stonefish's console application cannot construct camera sensors without
  # a rendering context.  The optional runtime gate stages a camera-free
  # fixture while validating every maintained world and all full scenes stay
  # covered by the structural test above.
  set(STONEFISH_NOGPU_BINARY "" CACHE FILEPATH
      "stonefish_simulator_nogpu executable used for scene parse validation")
  if(STONEFISH_NOGPU_BINARY)
    add_test(
      NAME uv_sim_assets_stonefish_parse
      COMMAND ${Python3_EXECUTABLE}
              ${CMAKE_CURRENT_SOURCE_DIR}/tools/validate_assets.py
              ${CMAKE_CURRENT_SOURCE_DIR}
              --stonefish-binary ${STONEFISH_NOGPU_BINARY}
    )
  endif()
endif()

ament_package()
