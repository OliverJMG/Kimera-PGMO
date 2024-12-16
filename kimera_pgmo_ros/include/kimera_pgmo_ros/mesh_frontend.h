/**
 * @file   mesh_frontend.h
 * @brief  MeshFrontend class: process incoming meshes
 * @author Yun Chang
 */
#pragma once

#include <kimera_pgmo_msgs/msg/kimera_pgmo_mesh.hpp>
#include <nav_interfaces/msg/pose_graph.hpp>
#include <rclcpp/rclcpp.hpp>

#include "kimera_pgmo/mesh_frontend_interface.h"

namespace kimera_pgmo {

class MeshFrontend : public rclcpp::Node, MeshFrontendInterface {
  friend class MeshFrontendTest;

 public:
  struct Config : MeshFrontendInterface::Config {
    int queue_size = 20;
    std::string frame_id = "world";
  } const config;

  /*! \brief Constructor for MeshFrontend class, which is in charge of
   * converting from mesh msg type to our mesh type while storing and
   * compressing the full unoptimized mesh
   */
  explicit MeshFrontend();

  virtual ~MeshFrontend() = default;

 protected:
  void handleMesh(const kimera_pgmo_msgs::msg::KimeraPgmoMesh::ConstSharedPtr& mesh);

  void publishFullMesh() const;

  void publishSimplifiedMesh() const;

  rclcpp::Subscription<kimera_pgmo_msgs::msg::KimeraPgmoMesh>::SharedPtr sub_;
  rclcpp::Publisher<kimera_pgmo_msgs::msg::KimeraPgmoMesh>::SharedPtr full_pub_;
  rclcpp::Publisher<kimera_pgmo_msgs::msg::KimeraPgmoMesh>::SharedPtr simplified_pub_;
  //! publish the factors corresponding to the edges of the simplified mesh
  rclcpp::Publisher<nav_interfaces::msg::PoseGraph>::SharedPtr mesh_graph_pub_;
};

void declare_config(MeshFrontend::Config& config);

}  // namespace kimera_pgmo
