/**
 * @file   kimera_pgmo_node.cpp
 * @brief  Main load for kimera pgmo
 * @author Yun Chang
 */
#include <rclcpp/rclcpp.hpp>

#include "kimera_pgmo_ros/kimera_pgmo.h"
#include "kimera_pgmo_ros/ros_log_sink.h"

int main(int argc, char* argv[]) {
  // Initialize ROS node.
  rclcpp::init(argc, argv);

  logging::Logger::addSink("ros", std::make_shared<kimera_pgmo::RosLogSink>());

  auto kimera_pgmo = std::make_shared<kimera_pgmo::KimeraPgmo>();
  if (!kimera_pgmo->initFromRos()) {
    RCLCPP_ERROR(rclcpp::get_logger("kimera_pgmo_node"), "Failed to initialize Kimera Pgmo.");
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  rclcpp::spin(kimera_pgmo);
  rclcpp::shutdown();

  return EXIT_SUCCESS;
}
