cmake_minimum_required(VERSION 3.12)
project(kimera_pgmo_rviz)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-Wall -Wextra)

find_package(ament_cmake_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(kimera_pgmo_msgs REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)

# from
# https://github.com/ros-visualization/visualization_tutorials/blob/noetic-devel/rviz_plugin_tutorials/CMakeLists.txt
set(CMAKE_AUTOMOC ON)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
QT5_WRAP_CPP(MOC_FILES
  include/kimera_pgmo_rviz/mesh_display.h
  include/kimera_pgmo_rviz/visibility_field.h
)
# set(QT_LIBRARIES Qt5::Widgets)

add_library(
  ${PROJECT_NAME}
  src/mesh_display.cpp
  src/mesh_visual.cpp
  src/tf_event_buffer.cpp
  src/visibility_field.cpp
  src/visuals_map.cpp
  ${MOC_FILES})
target_include_directories(${PROJECT_NAME} PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Qt5Widgets_INCLUDE_DIRS})

# target_compile_options(${PROJECT_NAME} PRIVATE -Wno-register
#                                                -Wno-deprecated-copy)
# target_link_options(${PROJECT_NAME} PRIVATE -Wl,--no-undefined)
ament_target_dependencies(${PROJECT_NAME} 
  pluginlib
  rviz_common
  rviz_ogre_vendor
  rviz_rendering
  kimera_pgmo_msgs)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(FILES plugins.xml DESTINATION share/${PROJECT_NAME})
install(DIRECTORY icons/ DESTINATION share/${PROJECT_NAME}/icons)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME})
pluginlib_export_plugin_description_file(rviz_common plugins.xml)

ament_package()