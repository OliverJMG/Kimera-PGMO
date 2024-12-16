cmake_minimum_required(VERSION 3.5)
project(kimera_pgmo_msgs)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/AbsolutePoseStamped.msg" 
  "msg/KimeraPgmoMesh.msg"
  "msg/KimeraPgmoMeshDelta.msg"
  "msg/TriangleIndices.msg")

set(srv_files
  "srv/LoadGraphMesh.srv" 
  "srv/RequestMeshFactors.srv")

rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
  ${msg_files}
  DEPENDENCIES geometry_msgs nav_interfaces std_msgs
)

ament_export_dependencies(rosidl_default_runtime geometry_msgs nav_interfaces std_msgs)

ament_package()
