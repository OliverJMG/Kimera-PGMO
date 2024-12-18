/**
 * @file   kimera_pgmo_node.cpp
 * @brief  Main load for kimera pgmo
 * @author Yun Chang
 */

#include <config_utilities/config_utilities.h>
#include <config_utilities/parsing/ros2.h>
#include <config_utilities/types/path.h>
#include <kimera_pgmo/utils/mesh_io.h>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>

#include <filesystem>

#include "kimera_pgmo_ros/conversion/mesh_conversion.h"

namespace kimera_pgmo {

using kimera_pgmo_msgs::msg::KimeraPgmoMesh;
using std::placeholders::_1;
using std::placeholders::_2;

class MeshPublisherNode : public rclcpp::Node {
 public:
  struct Config {
    size_t robot_id = 0;
    std::string mesh_frame = "map";
    std::filesystem::path mesh_filepath;
  } const config;

  explicit MeshPublisherNode();
  ~MeshPublisherNode() = default;

 private:
  bool reload(const std_srvs::srv::Empty::Request::SharedPtr request, 
          std_srvs::srv::Empty::Response::SharedPtr response);
  void publishMesh();

  rclcpp::Publisher<KimeraPgmoMesh>::SharedPtr pub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reload_service_;
};

void declare_config(MeshPublisherNode::Config& config) {
  using namespace config;
  name("MeshPublisherNode::Config");
  field(config.robot_id, "robot_id");
  field(config.mesh_frame, "mesh_frame");
  field<Path>(config.mesh_filepath, "mesh_filepath");
  checkCondition(!config.mesh_frame.empty(), "mesh_frame");
  check<Path::Exists>(config.mesh_filepath, "mesh_filepath");
}

MeshPublisherNode::MeshPublisherNode()
    : Node("mesh_publisher_node"), config(config::checkValid(config::fromRos<Config>(this->shared_from_this()))) {
  RCLCPP_INFO_STREAM(get_logger(), "Starting publisher node with\n" << config::toString(config));
  pub_ = this->create_publisher<KimeraPgmoMesh>("mesh", 1);
  publishMesh();

  reload_service_ = this->create_service<std_srvs::srv::Empty>("reload", 
          std::bind(&MeshPublisherNode::reload, this, _1, _2));
}

bool MeshPublisherNode::reload(const std_srvs::srv::Empty::Request::SharedPtr request, 
          std_srvs::srv::Empty::Response::SharedPtr response) {
  publishMesh();
  return true;
}

void MeshPublisherNode::publishMesh() {
  RCLCPP_INFO_STREAM(get_logger(), "Loading mesh from: " << config.mesh_filepath);
  pcl::PolygonMesh mesh;
  std::vector<Timestamp> stamps;
  ReadMeshWithStampsFromPly(config.mesh_filepath, mesh, &stamps);
  const auto num_vertices = mesh.cloud.height * mesh.cloud.width;
  RCLCPP_INFO_STREAM(get_logger(), "Loaded mesh with " << num_vertices << " vertices, "
                                      << mesh.polygons.size() << " faces, and "
                                      << stamps.size() << " timestamps");

  auto msg = conversions::toMsg(config.robot_id, mesh, stamps, config.mesh_frame);
  // ROS_ASSERT_MSG(msg != nullptr, "valid mesh required");
  msg->header.stamp = get_clock()->now();
  pub_->publish(*msg);
}

}  // namespace kimera_pgmo

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<kimera_pgmo::MeshPublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
