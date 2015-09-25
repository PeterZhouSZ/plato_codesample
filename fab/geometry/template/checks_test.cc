#include <Eigen/Dense>
using namespace Eigen;

#include <boost/smart_ptr/scoped_ptr.hpp>
#include <gflags/gflags.h>
#include <vector>
using std::vector;

#include <gtest/gtest.h>
#include <glog/logging.h>

#include <boost/smart_ptr/scoped_ptr.hpp>
using boost::scoped_ptr;

#include "fab/geometry/convert.h"
#include "fab/geometry/template/checks.h"
#include "fab/geometry/template/shape.h"
#include "fab/geometry/tri_mesh.h"

using mit_plato::Shape;
using mit_plato::ShapeChecker;

DECLARE_double(shape_volume_precision);

namespace {

void CreateTet(TriMesh* mesh, bool closed) {
  vector<TriMesh::VertexHandle> handles;
  handles.push_back(mesh->add_vertex(TriMesh::Point(0, 0, 0)));
  handles.push_back(mesh->add_vertex(TriMesh::Point(1, 0, 0)));
  handles.push_back(mesh->add_vertex(TriMesh::Point(0, 0, 1)));
  handles.push_back(mesh->add_vertex(TriMesh::Point(0, 1, 0)));

  mesh->add_face(handles[0], handles[1], handles[2]);
  mesh->add_face(handles[0], handles[2], handles[3]);
  mesh->add_face(handles[0], handles[3], handles[1]);
  if (closed)
    mesh->add_face(handles[1], handles[3], handles[2]);
}

void AddFaces(TriMesh* mesh,
              const vector<TriMesh::VertexHandle> handles,
              int i0, int i1, int i2, int i3) {
  mesh->add_face(handles[i0], handles[i1], handles[i2]);
  mesh->add_face(handles[i2], handles[i3], handles[i0]);
}

void CreateCuboid(TriMesh* mesh, double x = 0) {
  vector<TriMesh::VertexHandle> handles;
  handles.push_back(mesh->add_vertex(TriMesh::Point(x + 0, 0, 0)));
  handles.push_back(mesh->add_vertex(TriMesh::Point(x + 1, 0, 0)));
  handles.push_back(mesh->add_vertex(TriMesh::Point(x + 1, 0, 1)));
  handles.push_back(mesh->add_vertex(TriMesh::Point(x + 0, 0, 1)));

  handles.push_back(mesh->add_vertex(TriMesh::Point(x + 0, 2, 0)));
  handles.push_back(mesh->add_vertex(TriMesh::Point(x + 1, 2, 0)));
  handles.push_back(mesh->add_vertex(TriMesh::Point(x + 1, 2, 1)));
  handles.push_back(mesh->add_vertex(TriMesh::Point(x + 0, 2, 1)));

  AddFaces(mesh, handles, 0, 1, 2, 3);
  AddFaces(mesh, handles, 7, 6, 5, 4);
  AddFaces(mesh, handles, 1, 5, 6, 2);
  AddFaces(mesh, handles, 6, 7, 3, 2);
  AddFaces(mesh, handles, 4, 0, 3, 7);
  AddFaces(mesh, handles, 1, 0, 4, 5);
}

void CreateSkewedCuboid2(TriMesh* mesh, double height) {
  vector<TriMesh::VertexHandle> handles;
  handles.push_back(mesh->add_vertex(TriMesh::Point(0, 0, 0)));
  handles.push_back(mesh->add_vertex(TriMesh::Point(1, 0, 0)));
  handles.push_back(mesh->add_vertex(TriMesh::Point(1, 0, 1)));
  handles.push_back(mesh->add_vertex(TriMesh::Point(0, 0, 1)));

  handles.push_back(mesh->add_vertex(TriMesh::Point(0, 2, height)));
  handles.push_back(mesh->add_vertex(TriMesh::Point(1, 2, height)));
  handles.push_back(mesh->add_vertex(TriMesh::Point(1, 2, 1 + height)));
  handles.push_back(mesh->add_vertex(TriMesh::Point(0, 2, 1 + height)));

  AddFaces(mesh, handles, 0, 1, 2, 3);
  AddFaces(mesh, handles, 7, 6, 5, 4);
  AddFaces(mesh, handles, 1, 5, 6, 2);
  AddFaces(mesh, handles, 6, 7, 3, 2);
  AddFaces(mesh, handles, 4, 0, 3, 7);
  AddFaces(mesh, handles, 1, 0, 4, 5);
}

void CreateSkewedCuboid(TriMesh* mesh, double height) {
  vector<TriMesh::VertexHandle> handles;
  handles.push_back(mesh->add_vertex(TriMesh::Point(0, 0, 0)));
  handles.push_back(mesh->add_vertex(TriMesh::Point(1, 0, 0)));
  handles.push_back(mesh->add_vertex(TriMesh::Point(1, 0, 1)));
  handles.push_back(mesh->add_vertex(TriMesh::Point(0, 0, 1)));

  handles.push_back(mesh->add_vertex(TriMesh::Point(height, height, 0)));
  handles.push_back(mesh->add_vertex(TriMesh::Point(1 + height, height, 0)));
  handles.push_back(mesh->add_vertex(TriMesh::Point(1 + height, height, 1)));
  handles.push_back(mesh->add_vertex(TriMesh::Point(height, height, 1)));

  AddFaces(mesh, handles, 0, 1, 2, 3);
  AddFaces(mesh, handles, 7, 6, 5, 4);
  AddFaces(mesh, handles, 1, 5, 6, 2);
  AddFaces(mesh, handles, 6, 7, 3, 2);
  AddFaces(mesh, handles, 4, 0, 3, 7);
  AddFaces(mesh, handles, 1, 0, 4, 5);
}

}  // namespace

TEST(ShapeChecker, WatertightTest) {
  scoped_ptr<Shape> s(new Shape());
  CreateTet(&s->MutableMesh(), true);
  EXPECT_TRUE(ShapeChecker::IsWatertight(*s));

  s.reset(new Shape());
  CreateTet(&s->MutableMesh(), false);
  EXPECT_FALSE(ShapeChecker::IsWatertight(*s));
}

TEST(ShapeChecker, CenterOfMassTest) {
  scoped_ptr<Shape> s(new Shape());
  CreateCuboid(&s->MutableMesh());

  Eigen::Vector3d expected(0.5, 1.0, 0.5);
  Eigen::Vector3d actual = ShapeChecker::CenterOfMass(*s);
  EXPECT_TRUE(mds::ApproxEqual(expected, actual, 0.1))
      << "Expected: " << expected << "Actual: " << actual;
}

TEST(ShapeChecker, MaterialVolumeTest) {
  FLAGS_shape_volume_precision = 0.05;
  scoped_ptr<Shape> s(new Shape());
  CreateCuboid(&s->MutableMesh());

  double expected = 2.0;
  double actual = ShapeChecker::MaterialVolume(*s);
  EXPECT_TRUE(mds::ApproxEqual(expected, actual, 0.1))
      << "Expected: " << expected << "Actual: " << actual;
}

TEST(ShapeChecker, IsStableTest) {
  FLAGS_shape_volume_precision = 0.05;
  scoped_ptr<Shape> s(new Shape());
  CreateSkewedCuboid(&s->MutableMesh(), 0.5);

  EXPECT_TRUE(ShapeChecker::IsStable(*s));

  s.reset(new Shape());
  CreateSkewedCuboid(&s->MutableMesh(), 3.0);
  CHECK(OpenMesh::IO::write_mesh(s->Mesh(), "/tmp/checks2.obj"));
  EXPECT_FALSE(ShapeChecker::IsStable(*s));

  // Manually test on more complex models
  // s.reset(new Shape());
  // boost::scoped_ptr<TriMesh> mesh(
  //     CHECK_NOTNULL(TriMesh::read("/Users/shumash/Documents/3DModels/free_models/octopus.obj")));
  // s->MutableMesh() = *mesh;
  // EXPECT_FALSE(ShapeChecker::IsStable(*s));
}

TEST(ShapeChecker, IsInsideTest) {
  FLAGS_shape_volume_precision = 0.05;
  scoped_ptr<Shape> s(new Shape());
  CreateSkewedCuboid2(&s->MutableMesh(), 1.0);

  EXPECT_TRUE(ShapeChecker::IsInside(*s, Vector3d(0.5, 1, 0.6)));
  EXPECT_FALSE(ShapeChecker::IsInside(*s, Vector3d(0.5, 1, 0.6), 0.2));
  EXPECT_TRUE(ShapeChecker::IsInside(*s, Vector3d(0.7, 1, 1)));
  EXPECT_FALSE(ShapeChecker::IsInside(*s, Vector3d(0.7, 1, 1.6)));
}

TEST(ShapeChecker, GetShapeDeltaNorm) {
  scoped_ptr<Shape> s1(new Shape());
  CreateCuboid(&s1->MutableMesh());

  scoped_ptr<Shape> s2(new Shape());
  CreateCuboid(&s2->MutableMesh(), 0.5);

  double expected = 1 / 3.0;
  double actual = ShapeChecker::GetShapeDeltaNorm(*s1, *s2);
  EXPECT_TRUE(mds::ApproxEqual(expected, actual, 0.1))
      << "Expected: " << expected << "Actual: " << actual;
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  return RUN_ALL_TESTS();
}
