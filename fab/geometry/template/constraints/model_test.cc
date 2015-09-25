#include "fab/geometry/template/constraints/model.h"

#include <string>
#include <vector>
using std::string;
using std::vector;

#include <boost/smart_ptr/scoped_ptr.hpp>
#include <gecode/search.hh>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <OpenMesh/Core/IO/MeshIO.hh>

#include "fab/geometry/template/bounds.h"
#include "fab/geometry/template/editor.h"
#include "fab/geometry/template/params.h"
#include "fab/geometry/template/shape.h"
#include "fab/geometry/template/template.h"
#include "util/proto_util.h"


using namespace mit_plato;
using boost::scoped_ptr;

TEST(ModelTest, SimpleTest) {
  const string filename =
      "/Users/shumash/Documents/3DModels/templates/glass.ascii_proto";

  FabTemplateProto tpl_proto;
  CHECK(mds_util::ReadProtoFromFile(filename, &tpl_proto));
  FabGeometryTemplate* tpl = CHECK_NOTNULL(
      FabGeometryTemplate::FromSpec(tpl_proto));
  TemplateEditor editor(tpl);

  vector<VarHandle> handles;
  handles.push_back(VarHandle(
      "GlassBase",
      "base_curve:mit_plato.CreatePolygonSpec.spec:n_sides",
      VarHandleSpec::PARAM));  // $0
  handles.push_back(VarHandle(
      "GlassBody",
      "base_curve:mit_plato.CreatePolygonSpec.spec:n_sides",
      VarHandleSpec::PARAM));  // $1
  handles.push_back(VarHandle(
      "GlassBody",
      "vertical_curve:mit_plato.CreateHermiteSpec.spec:curve:control_pt#2:pt:y",
      VarHandleSpec::PARAM));  // $2
  CHECK(editor.AddConstraint("equal(2 * $0, $1)", handles));
  CHECK(editor.AddConstraint("equal($0 * 2, $2 * 3)", handles));
  CHECK(editor.params().MarkAsMutableInt(handles[0], new IntBounds(4, 15)));
  CHECK(editor.params().MarkAsMutableDouble(handles[2], new DoubleBounds(0.0, 20.0)));

  tpl_proto.Clear();
  CHECK(tpl->ToSpec(&tpl_proto));
  CHECK(mds_util::WriteASCIIProtoToFile(
      "/tmp/glass_constrained.ascii_proto", tpl_proto));

  tpl->SetParam<int32>(handles[1], 8);
  scoped_ptr<TemplateModel> model(new TemplateModel(tpl));
  Gecode::DFS<TemplateModel> solver(model.get());

  TemplateModel* solution = solver.next();
  if (solution) {
    LOG(INFO) << "Got solution!";
    solution->SetSolution(tpl);
    LOG(INFO) << "Set solution!";
    delete solution;

    // Write template to file
    CHECK(tpl->Instantiate());
    CHECK(OpenMesh::IO::write_mesh(tpl->shapes()[0]->Mesh(), "/tmp/glass.obj"));

    tpl_proto.Clear();
    CHECK(tpl->ToSpec(&tpl_proto));
    CHECK(mds_util::WriteASCIIProtoToFile(
        "/tmp/glass_constrained_solution.ascii_proto", tpl_proto));
  } else {
    LOG(FATAL) << "No solution";
  }
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  return RUN_ALL_TESTS();
}
