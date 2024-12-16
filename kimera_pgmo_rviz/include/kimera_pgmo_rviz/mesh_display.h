/**
 * @file   mesh_display.h
 * @brief  Rviz display for viewing mesh
 * @author Nathan Hughes
 */
#pragma once
#include <kimera_pgmo_msgs/msg/kimera_pgmo_mesh.hpp>
#include <rviz_common/message_filter_display.hpp>

#include <memory>

#include "kimera_pgmo_rviz/visuals_map.h"

namespace rviz_common::properties {
class BoolProperty;
}

namespace kimera_pgmo {

class MeshVisual;
class VisibilityField;
class TfEventBuffer;

class MeshDisplay
    : public rviz_common::MessageFilterDisplay<kimera_pgmo_msgs::msg::KimeraPgmoMesh> {
  Q_OBJECT
 public:
  MeshDisplay();
  ~MeshDisplay() override;

 protected:
  // Override Display functions.
  void onInitialize() override;

  void reset() override;

  void update(float /* wall_dt */, float /* ros_dt */) override;

 private Q_SLOTS:
   // Trigger an update of the settings for all visuals.
  void updateGlobalSettingsSlot();
  
  void visibleSlot();

  void toggleVisibilityAllSloT();

 private:
  void processMessage(const kimera_pgmo_msgs::msg::KimeraPgmoMesh::ConstSharedPtr msg) override;

  VisualsMap visuals_;

  // Getting transforms of visuals.
  std::unique_ptr<TfEventBuffer> tf_buffer_;

  // properties
  std::unique_ptr<rviz_common::properties::BoolProperty> cull_;
  std::unique_ptr<rviz_common::properties::BoolProperty> lighting_;
  // The root of the visibility tree.
  std::unique_ptr<VisibilityField> visibility_fields_;

  // Property to set visibility for all submaps.
  std::unique_ptr<rviz_common::properties::BoolProperty> toggle_visibility_all_property_;

  inline static const std::string kNsSeparator = "/";

 private:
  // Update the common settings of a single visual.
  void updateVisualSettings(MeshVisual& visual) const;

  MeshVisual* createVisual(const std::string& ns);
  void deleteVisual(const std::string& ns);

 private:
  // Interface for VisibilityFields to trigger visibility updates.
  friend class VisibilityField;
  void updateVisible();
};

}  // namespace kimera_pgmo
