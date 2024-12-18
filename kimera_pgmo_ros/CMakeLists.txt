cmake_minimum_required(VERSION 3.5)
project(kimera_pgmo_ros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-Wall -Wextra -Wno-sign-compare -Wno-unused-parameter)

find_package(config_utilities REQUIRED)
find_package(kimera_pgmo REQUIRED)
find_package(rclcpp REQUIRED)
find_package(kimera_pgmo_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_interfaces REQUIRED)
find_package(pose_graph_tools_ros REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)


add_library(${PROJECT_NAME}  
  src/conversion/gtsam_conversions.cpp
  src/kimera_pgmo.cpp
  src/conversion/mesh_conversion.cpp
  src/conversion/mesh_delta_conversion.cpp
  src/mesh_frontend.cpp
  src/visualization_functions.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/>)
target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC kimera_pgmo::kimera_pgmo 
  PRIVATE config_utilities::config_utilities)
ament_target_dependencies(
    ${PROJECT_NAME} PUBLIC
    kimera_pgmo_msgs
    nav_msgs
    std_srvs
    nav_interfaces
    pose_graph_tools_ros
    rclcpp
    tf2_ros
    visualization_msgs)

add_executable(kimera_pgmo_node app/kimera_pgmo_node.cpp)
target_link_libraries(kimera_pgmo_node ${PROJECT_NAME}
                      config_utilities::config_utilities)

add_executable(mesh_frontend_node app/mesh_frontend_node.cpp)
target_link_libraries(mesh_frontend_node ${PROJECT_NAME}
                      config_utilities::config_utilities)

add_executable(mesh_publisher_node app/mesh_publisher_node.cpp)
target_link_libraries(mesh_publisher_node ${PROJECT_NAME}
                      config_utilities::config_utilities)

add_executable(mesh_trajectory_deformer app/mesh_trajectory_deformer.cpp)
target_link_libraries(mesh_trajectory_deformer ${PROJECT_NAME}
                      config_utilities::config_utilities)

# Optionally build the voxblox frontend if voxblox_msgs is present.
find_package(voxblox_msgs QUIET)
if(voxblox_msgs_FOUND)
  message(STATUS "Building with voxblox_msgs support")
  add_executable(voxblox_mesh_frontend_node 
    app/voxblox_mesh_frontend_node.cpp 
    src/conversion/voxblox_msg_conversion.cpp
    src/voxblox_mesh_frontend.cpp
    src/voxblox_msg_interface.cpp
  )
  target_link_libraries(voxblox_mesh_frontend_node 
    PUBLIC ${PROJECT_NAME}
    PRIVATE config_utilities::config_utilities ${voxblox_msgs_LIBRARIES})
endif()

# if(BUILD_TESTING)
#   add_subdirectory(test)
# endif()

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  kimera_pgmo_msgs
  nav_msgs
  std_srvs
  nav_interfaces
  pose_graph_tools_ros
  visualization_msgs
)

install(
  TARGETS kimera_pgmo_node mesh_frontend_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ 
        DESTINATION include/\)

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}/)

ament_package()