/**
 * @file   KimeraPgmo.cpp
 * @brief  KimeraPgmo class: Main class and ROS interface
 * @author Yun Chang
 */
#include <chrono>
#include <cmath>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>

#include "kimera_pgmo/KimeraPgmo.h"

namespace kimera_pgmo {

// Constructor
KimeraPgmo::KimeraPgmo()
    : num_loop_closures_(0),
      inc_mesh_cb_time_(0),
      full_mesh_cb_time_(0),
      pg_cb_time_(0),
      path_cb_time_(0) {}

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

  // Log header to file
  if (log_output_) {
    std::string log_file = output_prefix_ + std::string("/kimera_pgmo_log.csv");
    logStats(log_file);
  }

  ROS_INFO("Initialized Kimera-PGMO.");

  return true;
}

// Load deformation parameters
bool KimeraPgmo::loadParameters(const ros::NodeHandle& n) {
  if (!KimeraPgmoInterface::loadParameters(n)) return false;

  if (!n.getParam("frame_id", frame_id_)) return false;
  if (!n.getParam("robot_id", robot_id_)) return false;
  if (n.getParam("output_prefix", output_prefix_)) {
    ROS_INFO("Saving optimized data to: %s/ mesh_pgmo.ply and traj_pgmo.csv",
             output_prefix_.c_str());
    n.getParam("log_output", log_output_);
    if (log_output_) {
      ROS_INFO("Logging output to: %s/kimera_pgmo_log.csv",
               output_prefix_.c_str());
    }
  }

  // start the mesh compression module for deformation graph
  double deformation_graph_resolution;
  if (!n.getParam("d_graph_resolution", deformation_graph_resolution))
    return false;

  compression_.reset(new OctreeCompression(deformation_graph_resolution));

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
  optimized_path_pub_ =
      nl.advertise<nav_msgs::Path>("optimized_path", 1, false);
  viz_deformation_graph_pub_ =
      nl.advertise<visualization_msgs::Marker>("deformation_graph", 10, false);
  return true;
}

// Initialize callbacks
bool KimeraPgmo::registerCallbacks(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  full_mesh_sub_ =
      nl.subscribe("full_mesh", 1, &KimeraPgmo::fullMeshCallback, this);

  incremental_mesh_sub_ = nl.subscribe(
      "incremental_mesh", 5, &KimeraPgmo::incrementalMeshCallback, this);

  pose_graph_incremental_sub_ =
      nl.subscribe("pose_graph_incremental",
                   1000,
                   &KimeraPgmo::incrementalPoseGraphCallback,
                   this);

  path_callback_sub_ =
      nl.subscribe("input_path", 2, &KimeraPgmo::optimizedPathCallback, this);

  // Initialize save mesh service
  save_mesh_srv_ =
      nl.advertiseService("save_mesh", &KimeraPgmo::saveMeshCallback, this);

  // Initialize save trajectory service
  save_traj_srv_ = nl.advertiseService(
      "save_trajectory", &KimeraPgmo::saveTrajectoryCallback, this);

  // Initialize request mesh edges service
  req_mesh_edges_srv_ = nl.advertiseService(
      "get_mesh_edges", &KimeraPgmo::requestMeshEdgesCallback, this);
  return true;
}

// To publish optimized mesh
bool KimeraPgmo::publishOptimizedMesh() const {
  std_msgs::Header msg_header;
  msg_header.stamp = last_mesh_stamp_;
  msg_header.frame_id = frame_id_;
  publishMesh(optimized_mesh_, msg_header, &optimized_mesh_pub_);
  return true;
}

// To publish optimized trajectory
bool KimeraPgmo::publishOptimizedPath() const {
  if (optimized_path_.size() == 0) return false;

  std_msgs::Header msg_header;
  msg_header.stamp = ros::Time::now();
  msg_header.frame_id = frame_id_;
  publishPath(optimized_path_, msg_header, &optimized_path_pub_);

  if (optimized_odom_pub_.getNumSubscribers() > 0) {
    // Publish also the optimized odometry
    nav_msgs::Odometry odometry_msg;
    const gtsam::Pose3 last_pose = optimized_path_[optimized_path_.size() - 1];
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
  if (msg->nodes.size() == 0 && msg->edges.size() == 0) return;
  // Start timer
  auto start = std::chrono::high_resolution_clock::now();

  processIncrementalPoseGraph(
      msg, &trajectory_, &unconnected_nodes_, &timestamps_);

  // Update transforms
  publishTransforms();

  // Update optimized path
  optimized_path_ = deformation_graph_.getOptimizedTrajectory(
      robot_id_to_prefix.at(robot_id_));

  // Stop timer and save
  auto stop = std::chrono::high_resolution_clock::now();
  auto spin_duration =
      std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  pg_cb_time_ = spin_duration.count();

  // Log to file
  if (log_output_) {
    std::string log_file = output_prefix_ + std::string("/kimera_pgmo_log.csv");
    logStats(log_file);
  }

  if (pose_graph_pub_.getNumSubscribers() > 0) {
    // Publish pose graph
    std::map<size_t, std::vector<ros::Time> > id_timestamps;
    id_timestamps[robot_id_] = timestamps_;
    const GraphMsgPtr& pose_graph_ptr =
        deformation_graph_.getPoseGraph(id_timestamps);
    pose_graph_pub_.publish(*pose_graph_ptr);
  }
}

void KimeraPgmo::optimizedPathCallback(
    const nav_msgs::Path::ConstPtr& path_msg) {
  // Start timer
  auto start = std::chrono::high_resolution_clock::now();

  if (trajectory_.size() > 1) {
    ROS_ERROR(
        "KimeraPgmo: Path subscriber does not support centralized multirobot "
        "scenario. ");
  }
  processOptimizedPath(path_msg, robot_id_);

  // Stop timer and save
  auto stop = std::chrono::high_resolution_clock::now();
  auto spin_duration =
      std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  path_cb_time_ = spin_duration.count();

  // Log to file
  if (log_output_) {
    std::string log_file = output_prefix_ + std::string("/kimera_pgmo_log.csv");
    logStats(log_file);
  }
}

void KimeraPgmo::fullMeshCallback(
    const kimera_pgmo::TriangleMeshIdStamped::ConstPtr& mesh_msg) {
  // Start timer
  auto start = std::chrono::high_resolution_clock::now();

  optimized_mesh_ = optimizeAndPublishFullMesh(mesh_msg, &optimized_mesh_pub_);

  // Stop timer and save
  auto stop = std::chrono::high_resolution_clock::now();
  auto spin_duration =
      std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  full_mesh_cb_time_ = spin_duration.count();

  return;
}

void KimeraPgmo::incrementalMeshCallback(
    const kimera_pgmo::TriangleMeshIdStamped::ConstPtr& mesh_msg) {
  // Start timer
  auto start = std::chrono::high_resolution_clock::now();

  processIncrementalMesh(
      mesh_msg, compression_, timestamps_, &unconnected_nodes_);

  // Stop timer and save
  auto stop = std::chrono::high_resolution_clock::now();
  auto spin_duration =
      std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  inc_mesh_cb_time_ = spin_duration.count();

  // Publish deformation graph visualization
  visualizeDeformationGraph(&viz_deformation_graph_pub_);

  return;
}

void KimeraPgmo::publishTransforms() {
  if (optimized_path_.size() == 0) return;

  const gtsam::Pose3& latest_pose =
      optimized_path_.at(optimized_path_.size() - 1);
  const gtsam::Point3& pos = latest_pose.translation();
  const gtsam::Quaternion& quat = latest_pose.rotation().toQuaternion();
  // Create transfomr message

  geometry_msgs::TransformStamped transform;
  std::string frame_name = "pgmo_base_link_";
  transform.header.stamp = ros::Time::now();
  transform.header.frame_id = "world";
  transform.child_frame_id = frame_name;
  transform.transform.translation.x = pos.x();
  transform.transform.translation.y = pos.y();
  transform.transform.translation.z = pos.z();
  transform.transform.rotation.x = quat.x();
  transform.transform.rotation.y = quat.y();
  transform.transform.rotation.z = quat.z();
  transform.transform.rotation.w = quat.w();

  tf_broadcast_.sendTransform(transform);
}

bool KimeraPgmo::saveMeshCallback(std_srvs::Empty::Request&,
                                  std_srvs::Empty::Response&) {
  // Save mesh
  std::string ply_name = output_prefix_ + std::string("/mesh_pgmo.ply");
  saveMesh(optimized_mesh_, ply_name);
  ROS_INFO("KimeraPgmo: Saved mesh to file.");
  return true;
}

bool KimeraPgmo::saveTrajectoryCallback(std_srvs::Empty::Request&,
                                        std_srvs::Empty::Response&) {
  // Save trajectory
  std::ofstream csvfile;
  std::string csv_name = output_prefix_ + std::string("/traj_pgmo.csv");
  saveTrajectory(optimized_path_, timestamps_, csv_name);
  ROS_INFO("KimeraPgmo: Saved trajectories to file.");
  return true;
}

bool KimeraPgmo::requestMeshEdgesCallback(
    kimera_pgmo::RequestMeshFactors::Request& request,
    kimera_pgmo::RequestMeshFactors::Response& response) {
  size_t offset_vertex_indices = 0;
  if (request.reindex_vertices) offset_vertex_indices = trajectory_.size();
  return getConsistencyFactors(
      request.robot_id, &response.mesh_factors, offset_vertex_indices);
}

void KimeraPgmo::logStats(const std::string filename) const {
  std::ofstream file;

  if (trajectory_.size() < 1) {
    file.open(filename);
    // file format
    file << "num-robots,num-keyframes,num-loop-closures,total-num-factors,num-"
            "vertices,num-vertices-simplified,inc-mesh-cb-time(mu-s),full-mesh-"
            "cb-time(mu-s),pg-cb-time(mu-s),path-cb-time(mu-s)\n";
    return;
  }
  // Number of keyframes
  size_t num_keyframes = trajectory_.size();
  // Number of vertices (total)
  size_t num_vertices =
      optimized_mesh_.cloud.width * optimized_mesh_.cloud.height;

  file.open(filename, std::ofstream::out | std::ofstream::app);
  file << 1 << "," << num_keyframes << "," << num_loop_closures_ << ","
       << deformation_graph_.getGtsamFactors().size() << "," << num_vertices
       << "," << deformation_graph_.getVertices().points.size() << ","
       << inc_mesh_cb_time_ << "," << full_mesh_cb_time_ << "," << pg_cb_time_
       << "," << path_cb_time_ << std::endl;
  file.close();
}

}  // namespace kimera_pgmo