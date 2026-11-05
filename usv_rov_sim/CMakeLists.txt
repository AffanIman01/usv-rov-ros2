cmake_minimum_required(VERSION 3.16)
project(usv_rov_sim LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(ament_cmake REQUIRED)
find_package(gz_sim_vendor REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz_transport_vendor REQUIRED)
find_package(gz_msgs_vendor REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

add_library(usv_rov_twin_thruster_controller SHARED src/twin_thruster_controller.cpp)
target_include_directories(usv_rov_twin_thruster_controller PRIVATE include)
target_link_libraries(usv_rov_twin_thruster_controller PRIVATE
  gz-sim8::gz-sim8 gz-plugin2::register gz-transport13::gz-transport13 gz-msgs10::gz-msgs10)
target_compile_options(usv_rov_twin_thruster_controller PRIVATE -Wall -Wextra -Wpedantic)
install(TARGETS usv_rov_twin_thruster_controller LIBRARY DESTINATION lib)
add_library(usv_rov_surface_buoyancy SHARED src/surface_buoyancy.cpp)
target_include_directories(usv_rov_surface_buoyancy PRIVATE include)
target_link_libraries(usv_rov_surface_buoyancy PRIVATE gz-sim8::gz-sim8 gz-plugin2::register)
target_compile_options(usv_rov_surface_buoyancy PRIVATE -Wall -Wextra -Wpedantic)
install(TARGETS usv_rov_surface_buoyancy LIBRARY DESTINATION lib)
add_executable(check_sim_plugins src/check_plugins.cpp)
target_link_libraries(check_sim_plugins PRIVATE gz-sim8::gz-sim8)
target_compile_options(check_sim_plugins PRIVATE -Wall -Wextra -Wpedantic)
install(TARGETS check_sim_plugins RUNTIME DESTINATION lib/${PROJECT_NAME})
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/gazebo_paths.dsv.in")
install(DIRECTORY launch config models worlds DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/keyboard_control.py DESTINATION lib/${PROJECT_NAME} RENAME keyboard_control)
install(PROGRAMS scripts/simulation_guard.py DESTINATION lib/${PROJECT_NAME} RENAME simulation_guard)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  enable_testing()
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_executable(test_thruster_control test/test_thruster_control.cpp)
  target_include_directories(test_thruster_control PRIVATE include)
  add_test(NAME thruster_control COMMAND test_thruster_control)
  add_executable(test_surface_buoyancy test/test_surface_buoyancy.cpp)
  target_include_directories(test_surface_buoyancy PRIVATE include)
  target_link_libraries(test_surface_buoyancy PRIVATE gz-sim8::gz-sim8)
  add_test(NAME surface_buoyancy COMMAND test_surface_buoyancy)
  add_test(NAME keyboard_input COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_keyboard.py)
  add_test(NAME simulation_guard COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_simulation_guard.py)
  add_test(NAME required_plugins_load COMMAND ${CMAKE_COMMAND} -E env
    "GZ_SIM_SYSTEM_PLUGIN_PATH=$<TARGET_FILE_DIR:usv_rov_surface_buoyancy>"
    $<TARGET_FILE:check_sim_plugins> ${CMAKE_CURRENT_SOURCE_DIR}/models/me120_dynamic/model.sdf)
  # An explicit missing filename must fail even when another workspace's library
  # happens to be discoverable. Exercise the same loader as the real model.
  file(READ "${CMAKE_CURRENT_SOURCE_DIR}/models/me120_dynamic/model.sdf" missing_plugin_model)
  string(REPLACE "filename=\"usv_rov_surface_buoyancy\""
    "filename=\"/__usv_rov_missing__/libusv_rov_surface_buoyancy.so\""
    missing_plugin_model "${missing_plugin_model}")
  file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/missing_plugin.sdf" "${missing_plugin_model}")
  add_test(NAME missing_plugin_rejected COMMAND check_sim_plugins
    "${CMAKE_CURRENT_BINARY_DIR}/missing_plugin.sdf")
  set_tests_properties(missing_plugin_rejected PROPERTIES WILL_FAIL TRUE)
  add_executable(test_buoyancy_lifecycle test/test_buoyancy_lifecycle.cpp)
  target_link_libraries(test_buoyancy_lifecycle PRIVATE gz-sim8::gz-sim8)
  add_test(NAME buoyancy_server_lifecycle COMMAND ${CMAKE_COMMAND} -E env
    "GZ_SIM_SYSTEM_PLUGIN_PATH=$<TARGET_FILE_DIR:usv_rov_surface_buoyancy>"
    $<TARGET_FILE:test_buoyancy_lifecycle>
    ${CMAKE_CURRENT_SOURCE_DIR}/models/me120_dynamic/model.sdf)
endif()
ament_package()
