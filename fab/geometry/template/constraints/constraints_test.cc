#include "fab/geometry/template/constraints/constraints.h"

#include <map>
using std::map;
#include <sstream>
using std::stringstream;

#include <boost/smart_ptr/scoped_ptr.hpp>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <gflags/gflags.h>
using boost::scoped_ptr;


#include "fab/geometry/template/params.h"

using namespace mit_plato;

TEST(ConstraintParser, SimpleTest) {
  SimpleContainer container;
  CHECK(container.AddVariable<double>(VarHandle("d0"), 1.5));
  CHECK(container.AddVariable<int32>(VarHandle("d1"), 1));
  CHECK(container.AddVariable<double>(VarHandle("d2"), 2.5));

  vector<VarHandle> handles;
  handles.push_back(VarHandle("d0"));  // $0
  handles.push_back(VarHandle("d1"));  // $1
  handles.push_back(VarHandle("d2"));  // $2

  stringstream ss("equal($0 + $1, $2); less_than($1, $0);");
  scoped_ptr<ConstraintParser<SimpleContainer> > parser(
      new ConstraintParser<SimpleContainer>(handles, &container, ss));

  scoped_ptr<Constraint> con(parser->ParseConstraint());
  ASSERT_TRUE(con != NULL);
  EXPECT_TRUE(con->IsSatisfied());
  LOG(INFO) << "Parsed constraint: " << con->ShortDescription();

  con.reset(parser->ParseConstraint());
  ASSERT_TRUE(con != NULL);
  EXPECT_TRUE(con->IsSatisfied());
  LOG(INFO) << "Parsed constraint: " << con->ShortDescription();
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  return RUN_ALL_TESTS();
}
