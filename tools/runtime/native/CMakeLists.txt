cmake_minimum_required(VERSION 3.16)
project(usv_rov_runtime_smoke LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
# Prefer ROS Jazzy's vendor exports when they are present; direct OSRF installs
# still work because these two lookups are optional.
find_package(gz_common_vendor QUIET)
find_package(gz_sim_vendor QUIET)
find_package(gz-common5 REQUIRED COMPONENTS graphics)
find_package(gz-sim8 REQUIRED)
add_executable(check_gazebo_meshes check_gazebo_meshes.cpp)
target_link_libraries(check_gazebo_meshes PRIVATE gz-common5::graphics)
add_executable(check_gazebo_world check_gazebo_world.cpp)
target_link_libraries(check_gazebo_world PRIVATE gz-sim8::gz-sim8)
