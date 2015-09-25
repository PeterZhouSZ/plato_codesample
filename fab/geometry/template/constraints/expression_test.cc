#include "fab/geometry/template/constraints/expression.h"

#include <map>
using std::map;
#include <sstream>
using std::stringstream;

#include <boost/smart_ptr/scoped_ptr.hpp>
#include <glog/logging.h>
#include <gtest/gtest.h>
using boost::scoped_ptr;

#include "fab/geometry/template/params.h"


using namespace mit_plato;

ExprNode* TestParseExpression(const vector<VarHandle>& handles,
                              SimpleContainer* container,
                              const string& expression) {
  LOG(INFO) << "Parsing expr: " << expression;
  stringstream ss;
  ss.str(expression);

  ExpressionParser<SimpleContainer> parser(handles, container, ss);
  ExprNode* expr = CHECK_NOTNULL(parser.Parse());
  LOG(INFO) << "Parsed expression: " << *expr;
  bool checks = expr->TypeChecks();
  if (checks) {
    LOG(INFO) << "Type checks ok";
  }
  EXPECT_TRUE(checks) << "Failed for expr: " << *expr;
  return expr;
}

void TestFailToParse(const vector<VarHandle>& handles,
                     SimpleContainer* container,
                     const string& expression) {
  LOG(INFO) << "Parsing [BAD] expr: " << expression;
  stringstream ss;
  ss.str(expression);

  ExpressionParser<SimpleContainer> parser(handles, container, ss);

  string error;
  ExprNode* expr = parser.Parse(&error);
  ASSERT_TRUE(expr == NULL) << "Erroneously parsed \"" << expression
                            << "\" as: " << *expr;
  LOG(INFO) << "Failed [CORRECTLY!] to parse with message: " << error;
}

TEST(ExpressionTest, ArithmeticParseTest) {
  SimpleContainer container;
  CHECK(container.AddVariable<double>(VarHandle("d0"), 15.75));
  CHECK(container.AddVariable<double>(VarHandle("d1"), 1.7));
  CHECK(container.AddVariable<double>(VarHandle("d2"), 2.5));
  CHECK(container.AddVariable<int32>(VarHandle("i0"), 7));
  CHECK(container.AddVariable<int32>(VarHandle("i1"), 2));
  CHECK(container.AddVariable<bool>(VarHandle("b0"), false));
  container.PrintInfo();

  vector<VarHandle> handles;
  handles.push_back(VarHandle("d0"));  // $0
  handles.push_back(VarHandle("d1"));  // $1
  handles.push_back(VarHandle("d2"));  // $2
  handles.push_back(VarHandle("i0"));  // $3
  handles.push_back(VarHandle("i1"));  // $4
  handles.push_back(VarHandle("b0"));  // $5

  // Double expressions --------------------------------------------------------
  // 15.75 + 1.7 * 2.5
  scoped_ptr<ExprNode> expr(TestParseExpression(handles, &container,
                                                "$0 + $1 * $2"));
  EXPECT_FLOAT_EQ(20.0, expr->EvaluateDouble());

  // 1.7 * 2.5 + 15.75 * 2 * 2
  expr.reset(TestParseExpression(handles, &container,
                                 "$1 * $2 + $0 * $4 * $4"));
  EXPECT_FLOAT_EQ(67.25, expr->EvaluateDouble());

  // 2.5 * 5.5
  expr.reset(TestParseExpression(handles, &container,
                                 "($2 * 5.5)"));
  EXPECT_FLOAT_EQ(13.75, expr->EvaluateDouble());

  // 2.5 * 5.5 + (15.75 + 7 + 1.5) * 8
  expr.reset(TestParseExpression(handles, &container,
                                 "($2 * 5.5) + ($0 + $3 + 1.5) * 8"));
  EXPECT_FLOAT_EQ(207.75, expr->EvaluateDouble());

  // 2.5 * 5.5 * -1.0 + (15.75 + 7 + 1.5) * 8
  expr.reset(TestParseExpression(handles, &container,
                                 "($2 * 5.5) * -1.0 + ($0 + $3 + 1.5) * 8  "));
  EXPECT_FLOAT_EQ(180.25, expr->EvaluateDouble());

  //  Gecode::LinIntExpr expr = expr->GetIntExpr(

  // Int expressions -----------------------------------------------------------
  // 7 + 200
  expr.reset(TestParseExpression(handles, &container, "($3 + 200)"));
  EXPECT_FLOAT_EQ(207, expr->EvaluateDouble());
  EXPECT_EQ(207, expr->EvaluateInt());

  // 200 / 7 (integer division)
  expr.reset(TestParseExpression(handles, &container, "200 / $3"));
  EXPECT_FLOAT_EQ(28, expr->EvaluateDouble());
  EXPECT_EQ(28, expr->EvaluateInt());

  // make sure non-inte division also works
  expr.reset(TestParseExpression(handles, &container, "200.0 / $3"));
  EXPECT_FLOAT_EQ(28.571428571428573, expr->EvaluateDouble());
  //  EXPECT_EQ(28, expr->EvaluateInt());  // supposed to throw

  // Failed expressions --------------------------------------------------------
  TestFailToParse(handles, &container, "$2 + * $3");
  TestFailToParse(handles, &container, "$2 + $31");
  TestFailToParse(handles, &container, "$2 + exp($1, $2)");
  TestFailToParse(handles, &container, "-($2 * 5.5)");
  TestFailToParse(handles, &container, "$2 * 5.5.6");
  TestFailToParse(handles, &container, "$2 ? 5");
  TestFailToParse(handles, &container, "$2 * $2 111");
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  return RUN_ALL_TESTS();
}
