cmake_minimum_required(VERSION 3.8)
project(usv_rov_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch rviz urdf meshes models worlds config
  DESTINATION share/${PROJECT_NAME}
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/README.md")
  install(FILES README.md DESTINATION share/${PROJECT_NAME})
endif()

ament_package()
