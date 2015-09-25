#ifndef _FAB_GEOMETRY_TEMPLATE_CONSTRAINTS_CONSTRAINTS_H__
#define _FAB_GEOMETRY_TEMPLATE_CONSTRAINTS_CONSTRAINTS_H__

#include <set>
#include <vector>
using std::set;
using std::vector;

#include <boost/thread/mutex.hpp>
#include <glog/logging.h>

#include "fab/geometry/template/constraints.pb.h"
#include "fab/geometry/template/expression.h"
#include "fab/geometry/template/params.h"
#include "util/stl_util.h"

namespace mit_plato {

class Constraint;
class TemplateModelHelper;

//! Handles reading constraints from spec and from the commandline
//! interpreter.
template <class Container>
class ConstraintParser : public ExpressionParser<Container> {
  // TODO: not ideal to inherit from ExpressionParser directly
 public:
  typedef typename ExpressionParser<Container>::Token Token;

  ConstraintParser(const vector<VarHandle>& handles,
                   const Container* container,
                   std::istream& constr_expr)
      : ExpressionParser<Container>(handles, container, constr_expr) {}

  Constraint* ParseConstraint(string* error_msg = NULL);

  // Throws errors
  Constraint* ParseConstraintUnsafe();

  static Constraint* ParseFromSpec(const RegisteredConstraintSpec& spec,
                                   const Container* container,
                                   string* error_msg = NULL);
};


//! Registers constraint classes by name.
class ConstraintRegister {
 public:
  typedef Constraint*
  (*CreateConstraintPtr)(const vector<ExprNode*>& expressions);

  static Constraint* Create(const string& name,
                            const vector<ExprNode*>& expresssions);

  static bool Register(const string& name,
                       CreateConstraintPtr create_fptr);

 private:
  ConstraintRegister() {}
  static ConstraintRegister& Singleton();

  std::map<string, CreateConstraintPtr> creators_;
  boost::mutex lock_;
};


// CONSTRAINT BASE -------------------------------------------------------------

//! Constraint class that operates on expression nodes;
//! each constraint should examine contained expressions to
//! determine if the types are compatible with the constraint.
class Constraint {
 public:
  // Takes ownership of expressions
  Constraint(const vector<ExprNode*>& expressions);

  virtual ~Constraint();

  bool ToEditableData(string* expression,
                      string* cond_expression,
                      vector<VarHandle>* handles) const;

  // Sets the condition under which this constraint is evaluated;
  // must be boolean; takes ownership.
  void SetCondition(ExprNode* condition);

  // Only works for constraints created via register, b/c register
  // know each constraint name, whereas the constraint itself does not.
  bool ToSpec(RegisteredConstraintSpec* spec) const;

  string ShortDescription() const;

  void GetUsedHandles(set<VarHandle>* handles) const;

  virtual bool IsSatisfied() const = 0;

  virtual void Post(TemplateModelHelper* helper) const = 0;

  // Returns true if no conditional expression is set.
  bool IsEnabled() const;

 protected:
  vector<ExprNode*> exps_;
  ExprNode* cond_exp_;

  // Get the common type to which all expressions evaluate
  static bool TypeCheckCommonTypes(const vector<ExprNode*>& exps,
                                   set<VarInfoSpec::VarType>* types);

  template <class T>
  void EvaluateAll(vector<T>* res) const;

  // Throws if fails
  void GetSingleSupportedType(VarInfoSpec::VarType* type) const;

 private:
  void set_given_name(const string& name) { given_name_ = name; }
  string given_name_;

  friend class ConstraintRegister;
};

// -----------------------------------------------------------------------------
// CONSTRAINT IMPLEMENTATIONS --------------------------------------------------
// -----------------------------------------------------------------------------

// EQUALITY --------------------------------------------------------------------
class EqualityConstraint : public Constraint {
 public:
  EqualityConstraint(const vector<ExprNode*>& expressions);

  virtual bool IsSatisfied() const;

  virtual void Post(TemplateModelHelper* helper) const;

 private:
  template <class T>
  bool AreTwoEqual(const T& v1, const T& v2) const {
    return v1 == v2;
  }

  template <class T>
  bool AreEqual(const vector<T>& res) const {
    if (res.size() < 2) return true;
    const T& v1 = res[0];
    for (int i = 1; i < res.size(); ++i) {
      if (!AreTwoEqual<T>(v1, res[i])) return false;
    }
    return true;
  }

  template <class T>
  bool IsSatisfiedTpl() const {
    vector<T> vals;
    EvaluateAll<T>(&vals);
    return AreEqual(vals);
  }

  VarInfo::VarType type_;
};

template <>
bool EqualityConstraint::AreTwoEqual<double>(const double& v1,
                                             const double& v2) const;


// LESS THAN -------------------------------------------------------------------
class LessThanConstraint : public Constraint {
 public:
  LessThanConstraint(const vector<ExprNode*>& expressions);

  virtual bool IsSatisfied() const;

  virtual void Post(TemplateModelHelper* helper) const;

 private:

  template <class T>
  bool IsSatisfiedTpl() const {
    vector<T> vals;
    EvaluateAll<T>(&vals);
    return vals[0] < vals[1];
  }

  VarInfo::VarType type_;
};


// -----------------------------------------------------------------------------
// DEFINITIONS
// -----------------------------------------------------------------------------
template <class Container>
Constraint* ConstraintParser<Container>::ParseFromSpec(
    const RegisteredConstraintSpec& spec,
    const Container* container,
    string* error_msg) {
  vector<VarHandle> handles;
  for (const VarHandleSpec& vspec : spec.var()) {
    handles.push_back(VarHandle(vspec));
  }

  vector<ExprNode*> expressions;
  ExprNode* cond_node = NULL;
  try {
    for (const string& exp_str : spec.expression()) {
      stringstream ss(exp_str);
      ExpressionParser<Container> parser(handles, container, ss);

      string error_str;
      ExprNode* node = parser.Parse(&error_str);
      if (!node) {
        throw runtime_error(error_str);
      }
      expressions.push_back(node);
    }

    if (!spec.conditional_expression().empty()) {
      stringstream ss(spec.conditional_expression());
      ExpressionParser<Container> parser(handles, container, ss);

      string error_str;
      cond_node = parser.Parse(&error_str);
      if (!cond_node) {
        throw runtime_error("Failed to parse conditional: " + error_str);
      }
    }
  } catch (runtime_error& e) {
    for (ExprNode* node : expressions) {
      delete node;
    }
    delete cond_node;
    LOG(ERROR) << "Failed to read constraint with error: " << e.what();
    if (error_msg) *error_msg = e.what();
    return NULL;
  }

  Constraint* constr = ConstraintRegister::Create(spec.type(), expressions);
  if (!constr) {
    LOG(ERROR) << "Could not create constraint from spec: '"
               << spec.DebugString() << "'";
    delete cond_node;
    if (error_msg)
      *error_msg = "Could not create constraint with name " + spec.type();
  }

  if (cond_node) {
    constr->SetCondition(cond_node);
  }
  return constr;
}

template <class Container>
Constraint* ConstraintParser<Container>::ParseConstraintUnsafe() {
  Token t = this->GetToken();
  if (t.content.empty()) {  // Nothing to parse
    return NULL;
  }

  if (t.type == ';') {
    return ParseConstraintUnsafe();
  }

  if (t.type != Token::ALPHA_TYPE) {
    throw runtime_error("Unexpected start characters: \"" + t.content +
                        "\". All constraints have the format "
                        "\"constraint_name(expr1, expr2, ...exprn).");
  }
  string constraint_name = t.content;
  t = this->GetToken();
  if (t.type != '(') {
    throw runtime_error("Unexpected characters '" + t.content +
                        "' after constraint name '" + constraint_name);
  }

  vector<ExprNode*> expressions;
  try {
    while (true) {
      ExprNode* node = this->GetExpression();
      expressions.push_back(node);
      t = this->GetToken();
      if (t.type != ',') break;
    }

    if (t.type != ')') {
      throw runtime_error("Expected ')' at end of expression");
    }

    t = this->GetToken();
    if (!t.content.empty() && t.type != ';') {
      throw runtime_error(
          "End of constraint expression expected before token " + t.content);
    }
  } catch (runtime_error& e) {
    for (ExprNode* node : expressions) {
      delete node;
    }
    throw e;
  }

  Constraint* constr = ConstraintRegister::Create(
      constraint_name, expressions);
  if (!constr) {
    throw runtime_error("Could not create constraint.");
  }
  return constr;
}

template <class Container>
Constraint* ConstraintParser<Container>::ParseConstraint(string* error_msg) {
  try {
    return ParseConstraintUnsafe();
  } catch (runtime_error& e) {
    LOG(ERROR) << e.what();
    if (error_msg) {
      *error_msg = e.what();
    }
    return NULL;
  }
}

}  // namespace mit_plato

#endif  // _FAB_GEOMETRY_TEMPLATE_CONSTRAINTS_CONSTRAINTS_H__
