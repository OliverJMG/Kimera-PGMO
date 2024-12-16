/**
 * @file   mesh_delta_conversion.h
 * @brief  Conversion to/from ROS
 * @author Nathan Hughes
 * @author Yun Chang
 */

#pragma once

#include <kimera_pgmo/mesh_delta.h>
#include <kimera_pgmo_msgs/msg/kimera_pgmo_mesh_delta.hpp>

namespace kimera_pgmo::conversions {

MeshDelta::Ptr fromMsg(const kimera_pgmo_msgs::msg::KimeraPgmoMeshDelta& msg);

kimera_pgmo_msgs::msg::KimeraPgmoMeshDelta::ConstSharedPtr toRosMsg(const MeshDelta& delta,
                                                         Timestamp timestamp_ns);

}  // namespace kimera_pgmo::conversions
