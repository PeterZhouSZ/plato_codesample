#include <gflags/gflags.h>
#include <gtest/gtest.h>
#include <glog/logging.h>
#include <OpenMesh/Core/IO/MeshIO.hh>

#include "fab/geometry/template/openscad_operations.h"
#include "fab/geometry/template/operations.pb.h"
#include "fab/geometry/template/shape.h"

using namespace mit_plato;

TEST(OpenscadCreate, SimpleTest) {
  RegisteredOpSpec spec;
  spec.MutableExtension(OpenscadCreateOpSpec::spec)->set_program("sphere(r=1);\n");

  Operation* op = CHECK_NOTNULL(Operation::FromSpec(spec));

  Shape* shape = dynamic_cast<CreateShapeOperation*>(op)->Create();
  CHECK(shape) << "Failed to create geometry";

  LOG(INFO) << shape->Mesh().n_vertices() << " vertices "
            << shape->Mesh().n_faces() << " faces";

  //  CHECK(OpenMesh::IO::write_mesh(shape->Mesh(), "/tmp/test.off"));
  delete shape;
}

TEST(OpenscadCreate, TestWithParams) {
  RegisteredOpSpec spec;
  OpenscadCreateOpSpec* scad_spec =
      spec.MutableExtension(OpenscadCreateOpSpec::spec);
  scad_spec->set_program(
      "module tire(R, width) {\n"
      //R = double, outer radius of the tire
      //w= double, width of the tire as it is rotated around; the width is equal to the thickness of the tire as well
      "r = R-width; \n" //r is the inner radius
      "s = width/2; \n"
      "curvature = s/2; \n"
      "rotate_extrude() \n"
      "translate ([r+curvature,-(s)/2,0]) \n"
      "minkowski(){square(s);circle(curvature);}\n"
      "}"
      "tire($0, $1);");

  MetavariableSpec* var = scad_spec->add_var();
  VarInfo info(VarHandle("R", VarHandleSpec::PARAM), VarInfoSpec::TYPE_DOUBLE);
  info.ToSpec(var->mutable_info());
  var->mutable_val()->set_double_val(20);

  var = scad_spec->add_var();
  info = VarInfo(VarHandle("w", VarHandleSpec::PARAM), VarInfoSpec::TYPE_DOUBLE);
  info.ToSpec(var->mutable_info());
  var->mutable_val()->set_double_val(2);

  Operation* op = CHECK_NOTNULL(Operation::FromSpec(spec));

  Shape* shape = dynamic_cast<CreateShapeOperation*>(op)->Create();
  CHECK(shape) << "Failed to create geometry";

  LOG(INFO) << shape->Mesh().n_vertices() << " vertices "
            << shape->Mesh().n_faces() << " faces";

  CHECK(OpenMesh::IO::write_mesh(shape->Mesh(), "/tmp/test_tire.off"));

  delete shape;
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  return RUN_ALL_TESTS();
}
