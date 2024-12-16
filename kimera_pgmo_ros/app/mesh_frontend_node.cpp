#include <rclcpp/rclcpp.hpp>

#include "kimera_pgmo_ros/mesh_frontend.h"
#include "kimera_pgmo_ros/ros_log_sink.h"

int main(int argc, char* argv[]) {
  // Initialize ROS node.
  rclcpp::init(argc, argv);

  logging::Logger::addSink("ros", std::make_shared<kimera_pgmo::RosLogSink>());

  auto mesh_frontend = std::make_shared<kimera_pgmo::MeshFrontend>();
  
  rclcpp::spin(mesh_frontend);
  rclcpp::shutdown();

  return EXIT_SUCCESS;
}
