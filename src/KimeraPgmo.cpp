/**
 * @file   KimeraPgmo.cpp
 * @brief  KimeraPgmo class: Main class and ROS interface
 * @author Yun Chang
 */
#include <geometry_msgs/PoseStamped.h>
#include <cmath>

#include "kimera_pgmo/KimeraPgmo.h"

namespace kimera_pgmo {

// Constructor
KimeraPgmo::KimeraPgmo() {}
KimeraPgmo::~KimeraPgmo() {}

// Initialize parameters, publishers, and subscribers and deformation graph
bool KimeraPgmo::initialize(const ros::NodeHandle& n) {
  if (!loadParameters(n)) {
    ROS_ERROR("KimeraPgmo: Failed to load parameters.");
  }

  if (!createPublishers(n)) {
    ROS_ERROR("KimeraPgmo: Failed to create publishers.");
  }

  if (!registerCallbacks(n)) {
    ROS_ERROR("KimeraPgmo: Failed to register callbacks.");
  }

  ROS_INFO("Initializes Mesher Mapper.");

  return true;
}

// Load deformation parameters
bool KimeraPgmo::loadParameters(const ros::NodeHandle& n) {
  if (!n.getParam("frame_id", frame_id_)) return false;
  if (!n.getParam("compression_time_horizon", compression_time_horizon_))
    return false;
  if (n.getParam("output_prefix", output_prefix_)) {
    ROS_INFO("Saving optimized data to: %s .ply and .csv",
             output_prefix_.c_str());
  }
  if (!n.getParam("embed_trajectory_delta_t", embed_delta_t_)) return false;

  // start the mesh compression module for deformation graph
  double deformation_graph_resolution;
  if (!n.getParam("d_graph_resolution", deformation_graph_resolution))
    return false;

  compression_.reset(new OctreeCompression(deformation_graph_resolution));

  // start deformation graph module
  double pgo_trans_threshold, pgo_rot_threshold;
  if (!n.getParam("rpgo/translation_threshold", pgo_trans_threshold))
    return false;
  if (!n.getParam("rpgo/rotation_threshold", pgo_rot_threshold)) return false;

  if (!deformation_graph_.initialize(pgo_trans_threshold, pgo_rot_threshold)) {
    ROS_ERROR("KimeraPgmo: Failed to initialize deformation graph.");
    return false;
  }
  return true;
}

// Initialize publishers
bool KimeraPgmo::createPublishers(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);
  optimized_mesh_pub_ =
      nl.advertise<mesh_msgs::TriangleMeshStamped>("optimized_mesh", 1, false);
  optimized_odom_pub_ =
      nl.advertise<nav_msgs::Odometry>("optimized_odom", 1, false);
  pose_graph_pub_ =
      nl.advertise<pose_graph_tools::PoseGraph>("pose_graph", 1, false);
  return true;
}

// Initialize callbacks
bool KimeraPgmo::registerCallbacks(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  full_mesh_sub_ =
      nl.subscribe("full_mesh", 5, &KimeraPgmo::fullMeshCallback, this);

  incremental_mesh_sub_ = nl.subscribe(
      "incremental_mesh", 5, &KimeraPgmo::incrementalMeshCallback, this);

  pose_graph_incremental_sub_ =
      nl.subscribe("pose_graph_incremental",
                   1000,
                   &KimeraPgmo::incrementalPoseGraphCallback,
                   this);

  optimized_path_pub_ =
      nl.advertise<nav_msgs::Path>("optimized_path", 1, false);

  // Initialize save mesh service
  save_mesh_srv_ =
      nl.advertiseService("save_mesh", &KimeraPgmo::saveMeshCallback, this);

  // Initialize save trajectory service
  save_traj_srv_ = nl.advertiseService(
      "save_trajectory", &KimeraPgmo::saveTrajectoryCallback, this);
  return true;
}

// To publish optimized mesh
bool KimeraPgmo::publishOptimizedMesh() {
  mesh_msgs::TriangleMesh mesh_msg =
      PolygonMeshToTriangleMeshMsg(optimized_mesh_);

  // Create msg
  mesh_msgs::TriangleMeshStamped new_msg;
  new_msg.header.stamp = last_mesh_stamp_;
  new_msg.header.frame_id = frame_id_;
  new_msg.mesh = mesh_msg;

  optimized_mesh_pub_.publish(new_msg);
  return true;
}

// To publish optimized trajectory
bool KimeraPgmo::publishOptimizedPath() const {
  std::vector<gtsam::Pose3> gtsam_path =
      deformation_graph_.getOptimizedTrajectory(GetRobotPrefix(robot_id_));

  if (gtsam_path.size() == 0) return false;

  if (optimized_path_pub_.getNumSubscribers() > 0) {
    // Create message type
    nav_msgs::Path path;

    // Fill path poses
    path.poses.reserve(gtsam_path.size());
    for (size_t i = 0; i < gtsam_path.size(); i++) {
      gtsam::Pose3 pose = gtsam_path.at(i);
      gtsam::Point3 trans = pose.translation();
      gtsam::Quaternion quat = pose.rotation().toQuaternion();

      geometry_msgs::PoseStamped ps_msg;
      ps_msg.header.frame_id = frame_id_;
      ps_msg.pose.position.x = trans.x();
      ps_msg.pose.position.y = trans.y();
      ps_msg.pose.position.z = trans.z();
      ps_msg.pose.orientation.x = quat.x();
      ps_msg.pose.orientation.y = quat.y();
      ps_msg.pose.orientation.z = quat.z();
      ps_msg.pose.orientation.w = quat.w();

      path.poses.push_back(ps_msg);
    }

    // Publish path message
    path.header.stamp = ros::Time::now();
    path.header.frame_id = frame_id_;
    optimized_path_pub_.publish(path);
  }

  if (optimized_odom_pub_.getNumSubscribers() > 0) {
    // Publish also the optimized odometry
    nav_msgs::Odometry odometry_msg;
    const gtsam::Pose3 last_pose = gtsam_path[gtsam_path.size() - 1];
    const gtsam::Rot3& rotation = last_pose.rotation();
    const gtsam::Quaternion& quaternion = rotation.toQuaternion();

    // Create header.
    odometry_msg.header.stamp = ros::Time::now();
    odometry_msg.header.frame_id = frame_id_;

    // Position
    odometry_msg.pose.pose.position.x = last_pose.x();
    odometry_msg.pose.pose.position.y = last_pose.y();
    odometry_msg.pose.pose.position.z = last_pose.z();

    // Orientation
    odometry_msg.pose.pose.orientation.w = quaternion.w();
    odometry_msg.pose.pose.orientation.x = quaternion.x();
    odometry_msg.pose.pose.orientation.y = quaternion.y();
    odometry_msg.pose.pose.orientation.z = quaternion.z();

    optimized_odom_pub_.publish(odometry_msg);
  }
  return true;
}

void KimeraPgmo::incrementalPoseGraphCallback(
    const pose_graph_tools::PoseGraph::ConstPtr& msg) {
  if (msg->edges.size() == 0) {
    ROS_WARN("No edges in incremental pose graph msg. ");
    return;
  }

  // if first node initialize
  //// Note that we assume for all node ids that the keys start with 0
  if (trajectory_.size() == 0 && msg->nodes[0].key == 0) {
    robot_id_ = msg->nodes[0].robot_id;
    gtsam::Symbol key_symb(GetRobotPrefix(robot_id_), 0);
    gtsam::Pose3 init_pose = RosToGtsam(msg->nodes[0].pose);
    // Initiate first node but do not add prior
    deformation_graph_.initFirstNode(key_symb.key(), init_pose, false);
    // Add to trajectory and timestamp map
    trajectory_.push_back(init_pose);
    timestamps_.push_back(msg->nodes[0].header.stamp);
    // Push node to queue to be connected to mesh vertices later
    unconnected_nodes_.push(key_symb.key());
    ROS_INFO("Initialized first node in pose graph. ");
  }

  try {
    for (pose_graph_tools::PoseGraphEdge pg_edge : msg->edges) {
      // Make sure robot id is as expected
      if (pg_edge.robot_from == robot_id_ && pg_edge.robot_to == robot_id_) {
        // Get edge information
        const gtsam::Pose3 measure = RosToGtsam(pg_edge.pose);
        const Vertex prev_node = pg_edge.key_from;
        const Vertex current_node = pg_edge.key_to;
        gtsam::Symbol from_key(GetRobotPrefix(robot_id_), prev_node);
        gtsam::Symbol to_key(GetRobotPrefix(robot_id_), current_node);

        if (pg_edge.type == pose_graph_tools::PoseGraphEdge::ODOM) {
          // odometry edge
          // Sanity check key node
          if (trajectory_.size() != current_node) {
            ROS_WARN(
                "New current node does not match current trajectory length. %d "
                "vs %d",
                trajectory_.size(),
                current_node);
          }
          // Calculate pose of new node
          gtsam::Pose3 new_pose = trajectory_[prev_node].compose(measure);
          // Add to trajectory and timestamp maps
          if (trajectory_.size() == current_node)
            trajectory_.push_back(new_pose);
          timestamps_.push_back(pg_edge.header.stamp);
          // Add new node to queue to be connected to mesh later
          unconnected_nodes_.push(to_key);
          // Add to deformation graph
          deformation_graph_.addNewBetween(from_key, to_key, measure, new_pose);
        } else if (pg_edge.type == pose_graph_tools::PoseGraphEdge::LOOPCLOSE) {
          // Loop closure edge
          // Add to deformation graph
          deformation_graph_.addNewBetween(from_key, to_key, measure);
          ROS_INFO(
              "KimeraPgmo: Loop closure detected between node %d and node %d.",
              prev_node,
              current_node);
        }
      } else {
        ROS_ERROR(
            "KimeraPgmo: Received pose graph edge with unexpected robot id. "
            "Note expects single robot for KimeraPgmo. For multi robot usage, "
            "check out KimeraPgmoMerger. ");
      }
    }
  } catch (const std::exception& e) {
    ROS_ERROR("Error in KimeraPgmo incrementalPoseGraphCallback. ");
    ROS_ERROR(e.what());
  }
}

void KimeraPgmo::fullMeshCallback(
    const mesh_msgs::TriangleMeshStamped::ConstPtr& mesh_msg) {
  input_mesh_ = TriangleMeshMsgToPolygonMesh(mesh_msg->mesh);
  last_mesh_stamp_ = mesh_msg->header.stamp;

  // Update optimized mesh
  try {
    optimized_mesh_ = deformation_graph_.deformMesh(input_mesh_);
  } catch (const std::out_of_range& e) {
    ROS_ERROR("Failed to deform mesh. Out of range error. ");
    optimized_mesh_ = input_mesh_;
  }
  if (optimized_mesh_pub_.getNumSubscribers() > 0) {
    publishOptimizedMesh();
  }
  if (pose_graph_pub_.getNumSubscribers() > 0) {
    // Publish pose graph
    std::map<size_t, std::vector<ros::Time> > id_stamp_map;
    id_stamp_map[robot_id_] = timestamps_;
    GraphMsgPtr pose_graph_ptr = deformation_graph_.getPoseGraph(id_stamp_map);
    pose_graph_pub_.publish(*pose_graph_ptr);
  }
  return;
}

void KimeraPgmo::incrementalMeshCallback(
    const mesh_msgs::TriangleMeshStamped::ConstPtr& mesh_msg) {
  pcl::PolygonMesh incremental_mesh =
      TriangleMeshMsgToPolygonMesh(mesh_msg->mesh);

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr new_vertices(
      new pcl::PointCloud<pcl::PointXYZRGBA>);
  std::vector<pcl::Vertices> new_triangles;
  std::vector<size_t> new_indices;

  double msg_time = mesh_msg->header.stamp.toSec();
  compression_->pruneStoredMesh(msg_time - compression_time_horizon_);
  compression_->compressAndIntegrate(
      incremental_mesh, new_vertices, &new_triangles, &new_indices, msg_time);

  deformation_graph_.updateMesh(*new_vertices, new_triangles);
  // Associate nodes to mesh
  while (!unconnected_nodes_.empty()) {
    gtsam::Symbol node = gtsam::Symbol(unconnected_nodes_.front());
    unconnected_nodes_.pop();
    size_t robot_id = robot_prefix_to_id.at(node.chr());
    if (timestamps_[node.index()].toSec() > msg_time - embed_delta_t_) {
      ROS_INFO("Connecting robot %d node %d to %d vertices. ",
               robot_id,
               node.index(),
               new_indices.size());
      deformation_graph_.addNodeValence(node, new_indices);
    }
    // termination guarantee
    if (timestamps_[node.index()].toSec() > msg_time + embed_delta_t_) break;
  }
  return;
}

bool KimeraPgmo::saveMeshCallback(std_srvs::Empty::Request&,
                                  std_srvs::Empty::Response&) {
  // Save mesh
  std::string ply_name = output_prefix_ + std::string(".ply");
  WriteMeshToPly(ply_name, optimized_mesh_);
  ROS_INFO("KimeraPgmo: Saved mesh to file.");
  return true;
}

bool KimeraPgmo::saveTrajectoryCallback(std_srvs::Empty::Request&,
                                        std_srvs::Empty::Response&) {
  // Save trajectory
  std::vector<gtsam::Pose3> optimized_path =
      deformation_graph_.getOptimizedTrajectory(GetRobotPrefix(robot_id_));
  std::ofstream csvfile;
  std::string csv_name = output_prefix_ + std::string(".csv");
  csvfile.open(csv_name);
  csvfile << "timestamp[ns],x,y,z,qw,qx,qy,qz\n";
  for (size_t i = 0; i < optimized_path.size(); i++) {
    gtsam::Point3 pos = optimized_path[i].translation();
    gtsam::Quaternion quat = optimized_path[i].rotation().toQuaternion();
    ros::Time stamp = timestamps_[i];
    csvfile << stamp.toNSec() << "," << pos.x() << "," << pos.y() << ","
            << pos.z() << "," << quat.w() << "," << quat.x() << "," << quat.y()
            << "," << quat.z() << "\n";
  }
  csvfile.close();
  ROS_INFO("KimeraPgmo: Saved trajectories to file.");
  return true;
}

}  // namespace kimera_pgmo