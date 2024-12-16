/**
 * @file   mesh_frontend.cpp
 * @brief  MeshFrontend class: process incoming meshes and sample it for
 * the mesh parts of the deformation graph
 * @author Yun Chang
 */
#include "kimera_pgmo_ros/mesh_frontend.h"

#include <config_utilities/config.h>
#include <config_utilities/parsing/ros2.h>
#include <kimera_pgmo/utils/common_functions.h>
#include <kimera_pgmo/utils/pcl_mesh_interface.h>
#include <nav_interfaces/msg/pose_graph.hpp>
#include <pose_graph_tools_ros/conversions.h>

#include <chrono>

#include "kimera_pgmo_ros/conversion/mesh_conversion.h"

namespace kimera_pgmo {

using kimera_pgmo_msgs::msg::KimeraPgmoMesh;
using nav_interfaces::msg::PoseGraph;

using std::placeholders::_1;

void declare_config(MeshFrontend::Config& config) {
  using namespace config;
  name("MeshFrontend::Config");
  base<MeshFrontendInterface::Config>(config);
  field(config.queue_size, "queue_size");
  field(config.frame_id, "frame_id");
}

MeshFrontend::MeshFrontend()
    : Node("mesh_frontend_node"), 
    MeshFrontendInterface(config::fromRos<kimera_pgmo::MeshFrontend::Config>(*this)), 
    config(config::fromRos<kimera_pgmo::MeshFrontend::Config>(*this)) {
  full_pub_ = create_publisher<KimeraPgmoMesh>("full_mesh", 1);
  simplified_pub_ = create_publisher<KimeraPgmoMesh>("deformation_graph_mesh", 10);
  mesh_graph_pub_ = create_publisher<PoseGraph>("mesh_graph_incremental", 100);
  sub_ = create_subscription<KimeraPgmoMesh>(
      "mesh_in", config.queue_size, std::bind(&MeshFrontend::handleMesh, this, _1));
  RCLCPP_INFO(get_logger(), "Initialized mesh_frontend.");
}

void MeshFrontend::handleMesh(const KimeraPgmoMesh::ConstSharedPtr& msg) {
  const pcl::PolygonMesh mesh = conversions::fromMsg(*msg);
  const PclMeshInterface mesh_interface(mesh);
  update(mesh_interface, rclcpp::Time(msg->header.stamp).seconds());

  // Publish edges and nodes if subscribed
  if (mesh_graph_pub_->get_subscription_count() > 0) {
    auto msg = pose_graph_tools::toMsg(*last_mesh_graph_);
    mesh_graph_pub_->publish(msg);
  }

  publishFullMesh();
  publishSimplifiedMesh();
}

void MeshFrontend::publishFullMesh() const {
  if (full_pub_->get_subscription_count() == 0) {
    return;
  }

  if (vertices_->size() == 0) {
    return;
  }

  auto msg = conversions::toMsg(config.robot_id,
                                *vertices_,
                                *triangles_,
                                *vertex_stamps_,
                                config.frame_id,
                                mesh_to_graph_idx_.get());

  full_pub_->publish(*msg);
}

void MeshFrontend::publishSimplifiedMesh() const {
  if (simplified_pub_->get_subscription_count() == 0) {
    return;
  }

  auto msg = conversions::toMsg(config.robot_id,
                                *graph_vertices_,
                                *graph_triangles_,
                                *graph_stamps_,
                                config.frame_id);

  simplified_pub_->publish(*msg);
}

}  // namespace kimera_pgmo
