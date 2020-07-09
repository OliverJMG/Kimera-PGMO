/**
 * @file   testDeformationGraph.cpp
 * @brief  Unit-tests for the deformation graph class
 * @author Yun Chang
 */

#include "gtest/gtest.h"

#include <geometry_msgs/Pose.h>
#include <gtsam/inference/Symbol.h>
#include <pcl/PolygonMesh.h>

#include "kimera_pgmo/DeformationGraph.h"
#include "kimera_pgmo/utils/CommonFunctions.h"
#include "test_config.h"

namespace kimera_pgmo {

pcl::PolygonMesh createMeshTriangle() {
  // Create simple pcl mesh with one triangle
  pcl::PolygonMesh mesh;

  pcl::PointCloud<pcl::PointXYZ> ptcld;
  ptcld.points.push_back(pcl::PointXYZ(0, 0, 0));
  ptcld.points.push_back(pcl::PointXYZ(1, 0, 0));
  ptcld.points.push_back(pcl::PointXYZ(0, 1, 0));
  pcl::toPCLPointCloud2(ptcld, mesh.cloud);

  pcl::Vertices tri_1;
  tri_1.vertices = std::vector<uint>{0, 1, 2};
  mesh.polygons = std::vector<pcl::Vertices>{tri_1};

  return mesh;
}

pcl::PolygonMesh SimpleMesh() {
  // Create simple pcl mesh
  pcl::PolygonMesh mesh;

  pcl::PointCloud<pcl::PointXYZRGBA> ptcld;
  pcl::PointXYZRGBA v0, v1, v2, v3, v4;
  v0.x = 0;
  v0.y = 0;
  v0.z = 0;
  v0.r = 23;
  v0.g = 24;
  v0.b = 122;
  v0.a = 255;

  v1.x = 1;
  v1.y = 0;
  v1.z = 0;
  v1.r = 33;
  v1.g = 34;
  v1.b = 52;
  v1.a = 255;

  v2.x = 0;
  v2.y = 1;
  v2.z = 0;
  v2.r = 12;
  v2.g = 144;
  v2.b = 22;
  v2.a = 255;

  v3.x = 1;
  v3.y = 1;
  v3.z = 0;
  v3.r = 0;
  v3.g = 14;
  v3.b = 0;
  v3.a = 255;

  v4.x = 0;
  v4.y = 0;
  v4.z = 1;
  v4.r = 144;
  v4.g = 0;
  v4.b = 12;
  v4.a = 255;

  ptcld.points.push_back(v0);
  ptcld.points.push_back(v1);
  ptcld.points.push_back(v2);
  ptcld.points.push_back(v3);
  ptcld.points.push_back(v4);
  pcl::toPCLPointCloud2(ptcld, mesh.cloud);

  pcl::Vertices tri_1, tri_2, tri_3, tri_4;
  tri_1.vertices = std::vector<uint>{0, 1, 2};
  tri_2.vertices = std::vector<uint>{1, 3, 2};
  tri_3.vertices = std::vector<uint>{0, 1, 4};
  tri_4.vertices = std::vector<uint>{0, 4, 2};
  mesh.polygons = std::vector<pcl::Vertices>{tri_1, tri_2, tri_3, tri_4};

  return mesh;
}

bool ComparePointcloud(const pcl::PointCloud<pcl::PointXYZRGBA>& cloud1,
                       const pcl::PointCloud<pcl::PointXYZRGBA>& cloud2,
                       double precision = 0.0) {
  if (cloud1.points.size() != cloud2.points.size()) return false;
  for (size_t i = 0; i < cloud1.points.size(); i++) {
    pcl::PointXYZRGBA p1 = cloud1.points[i];
    pcl::PointXYZRGBA p2 = cloud2.points[i];
    if (abs(p1.x - p2.x) > precision) return false;
    if (abs(p1.y - p2.y) > precision) return false;
    if (abs(p1.z - p2.z) > precision) return false;
    if (p1.r != p2.r) return false;
    if (p1.g != p2.g) return false;
    if (p1.b != p2.b) return false;
    if (p1.a != p2.a) return false;
  }
  return true;
}

TEST(DeformationGraph, reconstructMesh) {
  DeformationGraph graph;
  graph.Initialize(100, 100);
  pcl::PolygonMesh simple_mesh = createMeshTriangle();

  pcl::PolygonMesh original_mesh = SimpleMesh();

  // deform mesh
  graph.updateMesh(simple_mesh);
  graph.update();

  // First try deform with k = 1, should not change
  pcl::PolygonMesh new_mesh = graph.deformMesh(original_mesh, 1);
  pcl::PointCloud<pcl::PointXYZRGBA> deformed_vertices;
  pcl::fromPCLPointCloud2(new_mesh.cloud, deformed_vertices);
  EXPECT_EQ(5, deformed_vertices.points.size());
  EXPECT_EQ(1, deformed_vertices.points[1].x);
  EXPECT_EQ(1, deformed_vertices.points[2].y);
  EXPECT_EQ(1, deformed_vertices.points[4].z);
  EXPECT_EQ(33, deformed_vertices.points[1].r);
  EXPECT_EQ(144, deformed_vertices.points[2].g);
  EXPECT_EQ(255, deformed_vertices.points[3].a);
  EXPECT_EQ(144, deformed_vertices.points[4].r);
  EXPECT_EQ(original_mesh.polygons[0].vertices, new_mesh.polygons[0].vertices);
  EXPECT_EQ(original_mesh.polygons[3].vertices, new_mesh.polygons[3].vertices);

  // Try with k = 2
  new_mesh = graph.deformMesh(original_mesh, 2);
  pcl::fromPCLPointCloud2(new_mesh.cloud, deformed_vertices);
  EXPECT_EQ(5, deformed_vertices.points.size());
  EXPECT_EQ(1, deformed_vertices.points[1].x);
  EXPECT_EQ(1, deformed_vertices.points[2].y);
  EXPECT_EQ(1, deformed_vertices.points[4].z);
  EXPECT_EQ(33, deformed_vertices.points[1].r);
  EXPECT_EQ(144, deformed_vertices.points[2].g);
  EXPECT_EQ(255, deformed_vertices.points[3].a);
  EXPECT_EQ(144, deformed_vertices.points[4].r);
  EXPECT_EQ(original_mesh.polygons[0].vertices, new_mesh.polygons[0].vertices);
  EXPECT_EQ(original_mesh.polygons[3].vertices, new_mesh.polygons[3].vertices);
}

TEST(DeformationGraph, deformMeshtranslation) {
  DeformationGraph graph;
  graph.Initialize(100, 100);
  pcl::PolygonMesh simple_mesh = createMeshTriangle();

  pcl::PolygonMesh original_mesh = SimpleMesh();

  // deform mesh
  graph.updateMesh(simple_mesh);
  geometry_msgs::Pose distortion;
  distortion.position.x = 1.5;
  graph.update();
  graph.addMeasurement(1, distortion);

  pcl::PointCloud<pcl::PointXYZRGBA> original_vertices, expected_vertices;
  pcl::fromPCLPointCloud2(original_mesh.cloud, original_vertices);
  for (pcl::PointXYZRGBA p : original_vertices.points) {
    pcl::PointXYZRGBA new_point;
    new_point.x = p.x + 0.5;
    new_point.y = p.y;
    new_point.z = p.z;
    new_point.r = p.r;
    new_point.g = p.g;
    new_point.b = p.b;
    new_point.a = p.a;
    expected_vertices.push_back(new_point);
  }
  // First try deform with k = 1, should not change
  pcl::PolygonMesh new_mesh = graph.deformMesh(original_mesh, 1);
  pcl::PointCloud<pcl::PointXYZRGBA> actual_vertices;
  pcl::fromPCLPointCloud2(new_mesh.cloud, actual_vertices);

  EXPECT_TRUE(ComparePointcloud(expected_vertices, actual_vertices, 1e-6));
  EXPECT_EQ(original_mesh.polygons[0].vertices, new_mesh.polygons[0].vertices);
  EXPECT_EQ(original_mesh.polygons[3].vertices, new_mesh.polygons[3].vertices);

  // Try with k = 2
  new_mesh = graph.deformMesh(original_mesh, 2);
  pcl::fromPCLPointCloud2(new_mesh.cloud, actual_vertices);
  EXPECT_TRUE(ComparePointcloud(expected_vertices, actual_vertices, 1e-6));
  EXPECT_EQ(original_mesh.polygons[0].vertices, new_mesh.polygons[0].vertices);
  EXPECT_EQ(original_mesh.polygons[3].vertices, new_mesh.polygons[3].vertices);
}

TEST(DeformationGraph, deformMesh) {
  pcl::PolygonMeshPtr cube_mesh(new pcl::PolygonMesh());
  ReadMeshFromPly(std::string(DATASET_PATH) + "/cube.ply", cube_mesh);
  pcl::PolygonMesh simple_mesh = createMeshTriangle();
  // deform mesh
  DeformationGraph graph;
  graph.Initialize(100, 100);
  graph.updateMesh(simple_mesh);
  geometry_msgs::Pose distortion;
  distortion.position.x = -0.5;
  graph.update();
  graph.addMeasurement(0, distortion);
  pcl::PointCloud<pcl::PointXYZRGBA> original_vertices, expected_vertices;
  pcl::fromPCLPointCloud2(cube_mesh->cloud, original_vertices);
  for (pcl::PointXYZRGBA p : original_vertices.points) {
    pcl::PointXYZRGBA new_point;
    new_point.x = p.x - 0.5;
    new_point.y = p.y;
    new_point.z = p.z;
    new_point.r = p.r;
    new_point.g = p.g;
    new_point.b = p.b;
    new_point.a = p.a;
    expected_vertices.push_back(new_point);
  }
  // Try with k = 3
  pcl::PolygonMesh new_mesh = graph.deformMesh(*cube_mesh, 3);
  pcl::PointCloud<pcl::PointXYZRGBA> actual_vertices;
  pcl::fromPCLPointCloud2(new_mesh.cloud, actual_vertices);
  EXPECT_TRUE(ComparePointcloud(expected_vertices, actual_vertices, 1e-6));
  EXPECT_EQ(cube_mesh->polygons[0].vertices, new_mesh.polygons[0].vertices);
  EXPECT_EQ(cube_mesh->polygons[3].vertices, new_mesh.polygons[3].vertices);

  // deform mesh again
  geometry_msgs::Pose distortion2;
  distortion2.position.x = 1.5;
  graph.addMeasurement(1, distortion2);
  // Try with k = 3
  new_mesh = graph.deformMesh(*cube_mesh, 2);
  pcl::fromPCLPointCloud2(new_mesh.cloud, actual_vertices);
  EXPECT_NEAR(-0.5, actual_vertices.points[0].x, 0.001);
  EXPECT_NEAR(1.273, actual_vertices.points[1].x, 0.001);

  EXPECT_EQ(cube_mesh->polygons[0].vertices, new_mesh.polygons[0].vertices);
  EXPECT_EQ(cube_mesh->polygons[3].vertices, new_mesh.polygons[3].vertices);
}

TEST(DeformationGraph, updateMesh) {
  DeformationGraph graph;
  graph.Initialize(100, 100);
  pcl::PolygonMesh simple_mesh = createMeshTriangle();

  pcl::PolygonMesh original_mesh = SimpleMesh();

  graph.updateMesh(simple_mesh);
  graph.update();

  EXPECT_EQ(3, graph.getNumVertices());
  EXPECT_EQ(0, graph.getVertices().points[0].x);
  EXPECT_EQ(1, graph.getVertices().points[2].y);

  Vertices new_node_valences{0, 2};
  graph.addNode(pcl::PointXYZ(2, 2, 2), new_node_valences);
  graph.update();

  EXPECT_EQ(4, graph.getNumVertices());
  EXPECT_EQ(2, graph.getVertices().points[3].y);
  Graph base_graph = graph.getGraph();
  Vertex new_key = gtsam::Symbol('n', 0).key();

  std::vector<Edge> base_edges = base_graph.getEdges();
  EXPECT_EQ(10, base_edges.size());
  EXPECT_EQ(Edge(0, 1), base_edges[0]);
  EXPECT_EQ(Edge(0, new_key), base_edges[2]);
  EXPECT_EQ(Edge(1, 2), base_edges[4]);
  EXPECT_EQ(Edge(new_key, 2), base_edges[9]);

  Vertices new_node_valences_2{3};
  graph.addNode(pcl::PointXYZ(2, 3, 4), new_node_valences_2, true);
  graph.updateMesh(original_mesh);
  graph.update();

  EXPECT_EQ(7, graph.getNumVertices());
  EXPECT_EQ(2, graph.getVertices().points[5].y);
  EXPECT_EQ(3, graph.getVertices().points[6].y);
  EXPECT_EQ(4, graph.getVertices().points[6].z);
  base_graph = graph.getGraph();
  Vertex new_key_2 = gtsam::Symbol('n', 1).key();

  base_edges = base_graph.getEdges();
  EXPECT_EQ(24, base_edges.size());
  EXPECT_EQ(Edge(0, 1), base_edges[0]);
  EXPECT_EQ(Edge(0, new_key), base_edges[3]);
  EXPECT_EQ(Edge(1, 0), base_edges[4]);
  EXPECT_EQ(Edge(3, new_key_2), base_edges[15]);
  EXPECT_EQ(Edge(new_key, new_key_2), base_edges[21]);
}

TEST(DeformationGraph, addNodeMeasurement) {
  DeformationGraph graph;
  graph.Initialize(100, 100);
  pcl::PolygonMesh simple_mesh = createMeshTriangle();

  pcl::PolygonMesh original_mesh = SimpleMesh();

  graph.updateMesh(simple_mesh);
  Vertices new_node_valences{0, 2};
  graph.addNode(pcl::PointXYZ(2, 2, 2), new_node_valences);
  graph.update();

  EXPECT_EQ(4, graph.getNumVertices());
  EXPECT_EQ(2, graph.getVertices().points[3].y);
  EXPECT_EQ(0, graph.getVertices().points[0].x);

  // Add node measurement
  graph.addNodeMeasurement(
      0, gtsam::Pose3(gtsam::Rot3(0, 0, 0, 1), gtsam::Point3(2, 2, 2)));
  pcl::PolygonMesh new_mesh = graph.deformMesh(simple_mesh, 1);
  pcl::PointCloud<pcl::PointXYZRGBA> actual_vertices;
  pcl::fromPCLPointCloud2(new_mesh.cloud, actual_vertices);

  EXPECT_NEAR(4, actual_vertices.points[0].x, 0.001);
  EXPECT_NEAR(4, actual_vertices.points[0].y, 0.001);
  EXPECT_NEAR(0, actual_vertices.points[0].z, 0.001);
  EXPECT_NEAR(3, actual_vertices.points[1].x, 0.001);
  EXPECT_NEAR(3, actual_vertices.points[2].y, 0.001);
  EXPECT_NEAR(0, actual_vertices.points[1].z, 0.001);
  EXPECT_NEAR(4, actual_vertices.points[2].x, 0.001);
}

TEST(DeformationGraph, addNewBetween) {
  DeformationGraph graph;
  graph.Initialize(100, 100);
  pcl::PolygonMesh simple_mesh = createMeshTriangle();

  pcl::PolygonMesh original_mesh = SimpleMesh();

  graph.updateMesh(simple_mesh);
  graph.update();

  EXPECT_EQ(3, graph.getNumVertices());
  EXPECT_EQ(0, graph.getVertices().points[0].x);
  EXPECT_EQ(1, graph.getVertices().points[2].y);

  Vertices new_node_valences{0, 2};
  graph.initFirstNode(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(2, 2, 2)));
  graph.updateNodeValence(0, new_node_valences);
  graph.update();

  EXPECT_EQ(4, graph.getNumVertices());
  EXPECT_EQ(2, graph.getVertices().points[3].y);
  Graph base_graph = graph.getGraph();
  Vertex new_key = gtsam::Symbol('n', 0).key();

  std::vector<Edge> base_edges = base_graph.getEdges();
  EXPECT_EQ(10, base_edges.size());
  EXPECT_EQ(Edge(0, 1), base_edges[0]);
  EXPECT_EQ(Edge(0, new_key), base_edges[2]);
  EXPECT_EQ(Edge(1, 2), base_edges[4]);
  EXPECT_EQ(Edge(new_key, 2), base_edges[9]);

  graph.addNewBetween(0,
                      1,
                      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 1, 2)),
                      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(2, 3, 4)));
  graph.updateMesh(original_mesh);
  graph.update();

  // Expect no change
  pcl::PolygonMesh new_mesh = graph.deformMesh(original_mesh, 1);
  pcl::PointCloud<pcl::PointXYZ> actual_vertices;
  pcl::fromPCLPointCloud2(new_mesh.cloud, actual_vertices);

  EXPECT_EQ(0.0, actual_vertices.points[0].x);
  EXPECT_EQ(1.0, actual_vertices.points[3].y);
  EXPECT_EQ(1.0, actual_vertices.points[4].z);

  std::vector<gtsam::Pose3> traj = graph.getOptimizedTrajectory();
  EXPECT_EQ(2, traj.size());
  EXPECT_TRUE(gtsam::assert_equal(
      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(2, 2, 2)), traj[0]));
  EXPECT_TRUE(gtsam::assert_equal(
      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(2, 3, 4)), traj[1]));

  graph.addNewBetween(1,
                      2,
                      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, -0.9, -1.9)),
                      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(3, 2.1, 2.1)));
  graph.addNewBetween(
      0, 2, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0)));
  graph.updateMesh(original_mesh);
  graph.update();

  traj = graph.getOptimizedTrajectory();
  EXPECT_EQ(3, traj.size());
  EXPECT_TRUE(gtsam::assert_equal(
      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(3, 2, 2)), traj[2], 0.05));
}

}  // namespace kimera_pgmo