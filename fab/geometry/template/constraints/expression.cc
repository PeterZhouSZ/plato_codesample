#include <sstream>

#include "fab/geometry/template/constraints/expression.h"
#include "fab/geometry/convert.h"

namespace mit_plato {

// ExprNode --------------------------------------------------------------------
ExprNode::ExprNode(const vector<ExprNode*>* children)
    : children_(*children) {}

ExprNode::~ExprNode() {
  for (ExprNode* child : children_) {
    delete child;
  }
}

string ExprNode::ToString() const {
  stringstream ss;
  print(ss);
  return ss.str();
}

int32 ExprNode::EvaluateInt() const {
  throw runtime_error("cannot evaluate int");
}

double ExprNode::EvaluateDouble() const {
  throw runtime_error("cannot evaluate double");
}

bool ExprNode::EvaluateBool() const {
  throw runtime_error("cannot evaluate bool");
}

void ExprNode::GetUsedHandles(set<VarHandle>* handles) const {
  for (const ExprNode* child : children_) {
    child->GetUsedHandles(handles);
  }
}

Gecode::LinIntExpr ExprNode::GetIntExpr(TemplateModelHelper* helper) const {
  throw runtime_error("cannot get int expression");
}

Gecode::LinFloatExpr ExprNode::GetDoubleExpr(TemplateModelHelper* helper) const {
  throw runtime_error("cannot get float expression");
}

Gecode::BoolExpr ExprNode::GetBoolExpr(TemplateModelHelper* helper) const {
  throw runtime_error("cannot get bool expression");
}

// Unary Function Node ---------------------------------------------------------
UnaryFunctionNode::UnaryFunctionNode(const string& op,
                                     ExprNode* child)
    : op_(op) {
  children_.push_back(child);
  CheckSanity();
}

Gecode::LinFloatExpr UnaryFunctionNode::GetDoubleExpr(
    TemplateModelHelper* helper) const {
  Gecode::LinFloatExpr e = children_[0]->GetDoubleExpr(helper);
  if (op_ == "sin") {
    return expr(helper->home(), Gecode::sin(e));
  } else if (op_ == "cos") {
    return expr(helper->home(), Gecode::cos(e));
  } else if (op_ == "tan") {
    return expr(helper->home(), Gecode::tan(e));
  } else if (op_ == "atan") {
    return expr(helper->home(), Gecode::atan(e));
  } else if (op_ == "abs") {
    return expr(helper->home(), Gecode::abs(e));
  }
  throw runtime_error("Unsupported type: " + op_);
}

double UnaryFunctionNode::EvaluateDouble() const {
  if (op_ == "sin") {
    return std::sin(children_[0]->EvaluateDouble());
  } else if (op_ == "cos") {
    return std::cos(children_[0]->EvaluateDouble());
  } else if (op_ == "tan") {
    return std::tan(children_[0]->EvaluateDouble());
  } else if (op_ == "atan") {
    return std::atan(children_[0]->EvaluateDouble());
  } else if (op_ == "abs") {
    return fabs(children_[0]->EvaluateDouble());
  }
  throw runtime_error("Unsupported type: " + op_);
}

void UnaryFunctionNode::CheckSanity() const {
  CHECK(op_ == "sin" ||
        op_ == "cos" ||
        op_ == "tan" ||
        op_ == "atan" ||
        op_ == "abs") << "Unsupported op: \"" << op_ << "\"";
}

bool UnaryFunctionNode::TypeChecks(set<VarInfoSpec::VarType>* types) {
  set<VarInfoSpec::VarType> child_types;
  if (!children_[0]->TypeChecks(&child_types)) return false;
  if (child_types.find(VarInfoSpec::TYPE_DOUBLE) == child_types.end()) {
    throw runtime_error(
        "Unary function " + op_ +
        " only implemented for double expressions; type check fails.");
  }

  types->insert(VarInfoSpec::TYPE_DOUBLE);
  return true;
}

void UnaryFunctionNode::print(std::ostream& out,
                              const vector<VarHandle>* handles) const {
  out << op_ << "(";
  children_[0]->print(out, handles);
  out << ")";
}


// ArithNode -------------------------------------------------------------------
ArithNode::ArithNode(const char op,
                     ExprNode* child1,
                     ExprNode* child2)
    : op_(op), type_(VarInfoSpec::TYPE_UNK) {
  children_.push_back(child1);
  children_.push_back(child2);
  CheckSanity();
}

Gecode::LinIntExpr ArithNode::GetIntExpr(TemplateModelHelper* helper) const {
  Gecode::LinIntExpr e0 = children_[0]->GetIntExpr(helper);
  Gecode::LinIntExpr e1 = children_[1]->GetIntExpr(helper);
  Gecode::LinIntExpr res;

  CHECK(Evaluate(e0, e1, &res));
  return res;
}

Gecode::LinFloatExpr ArithNode::GetDoubleExpr(TemplateModelHelper* helper) const {
  if (type_ == VarInfoSpec::TYPE_DOUBLE) {
    Gecode::LinFloatExpr e0 = children_[0]->GetDoubleExpr(helper);
    Gecode::LinFloatExpr e1 = children_[1]->GetDoubleExpr(helper);
    Gecode::LinFloatExpr res;

    CHECK(Evaluate(e0, e1, &res));
    return res;
  } else if (type_ == VarInfoSpec::TYPE_INT32) {
    // Need to channel
    VarHandle handle = helper->AuxDoubleHandle(ToString());
    if (helper->HasVar(handle)) {
      return Gecode::LinFloatExpr(helper->GetVarDouble(handle));
    } else {
      Gecode::IntVar int_var = Gecode::expr(helper->home(), GetIntExpr(helper));
      const Gecode::FloatVar& float_var =
          helper->NewAuxDouble(handle, int_var.min(), int_var.max());
      Gecode::channel(helper->home(), float_var, int_var);
      return Gecode::LinFloatExpr(float_var);
    }
  } else {
    throw runtime_error("ArithNode::GetDoubleExpr not implemented for type " +
                        VarInfoSpec::VarType_Name(type_));
  }
}

void ArithNode::CheckSanity() const {
  CHECK(op_ == '*' ||
        op_ == '/' ||
        op_ == '+' ||
        op_ == '-' ||
        op_ == '=' ||
        op_ == '<') << "Unsupported op: \"" << op_ << "\"";
  CHECK_EQ(2, children_.size());
}

int32 ArithNode::EvaluateInt() const {
  if (type_ != VarInfoSpec::TYPE_INT32) {
    throw runtime_error("Type mismatch : " +
                        VarInfoSpec::VarType_Name(type_) + " vs. " +
                        VarInfoSpec::VarType_Name(VarInfoSpec::TYPE_INT32));
  }

  int32 res;
  CHECK(Evaluate(children_[0]->EvaluateInt(),
                 children_[1]->EvaluateInt(),
                 &res));
  return res;
}

bool ArithNode::EvaluateBool() const {
  if (op_ != '=' && op_ != '<') {
    throw runtime_error("Not a boolean expression! " + string(1, op_));
  }

  if (type_ == VarInfoSpec::TYPE_INT32) {
    if (op_ == '=') {
      return children_[0]->EvaluateInt() ==
          children_[1]->EvaluateInt();
    } else if (op_ == '<') {
      return children_[0]->EvaluateInt() <
        children_[1]->EvaluateInt();
    }
  } else if (type_ == VarInfoSpec::TYPE_DOUBLE) {
    if (op_ == '=') {
      return mds::ApproxEqual(children_[0]->EvaluateDouble(),
                              children_[1]->EvaluateDouble(), 0.00001);
    } else if (op_ == '<') {
      return children_[0]->EvaluateDouble() <
        children_[1]->EvaluateDouble();
    }
  } else if (type_ == VarInfoSpec::TYPE_BOOL) {
    if (op_ == '=') {
      return children_[0]->EvaluateBool() ==
          children_[1]->EvaluateBool();
    } else {
      return children_[0]->EvaluateBool() ==
          children_[1]->EvaluateBool();
    }
  }

  throw runtime_error("Type mismatch : " +
                      VarInfoSpec::VarType_Name(type_) + " vs. " +
                      VarInfoSpec::VarType_Name(VarInfoSpec::TYPE_BOOL));
}

double ArithNode::EvaluateDouble() const {
  if (type_ == VarInfoSpec::TYPE_INT32) {
    return EvaluateInt();
  }

  if (type_ != VarInfoSpec::TYPE_DOUBLE) {
    LOG(ERROR) << "Type mismatch : "
        << VarInfoSpec::VarType_Name(type_) << " vs. "
        << VarInfoSpec::VarType_Name(VarInfoSpec::TYPE_DOUBLE)
    << " for expr " << *this;
    throw runtime_error("Type mismatch : " +
                        VarInfoSpec::VarType_Name(type_) + " vs. " +
                        VarInfoSpec::VarType_Name(VarInfoSpec::TYPE_DOUBLE));
  }

  double res;
  CHECK(Evaluate(children_[0]->EvaluateDouble(),
                 children_[1]->EvaluateDouble(),
                 &res));
  return res;
}

bool ArithNode::TypeChecks(std::set<VarInfoSpec::VarType>* types) {
  if (type_ == VarInfoSpec::TYPE_UNK) {
    set<VarInfoSpec::VarType> types0, types1;
    if (!children_[0]->TypeChecks(&types0)) return false;
    if (!children_[1]->TypeChecks(&types1)) return false;

    if (mds::Contains(types0, VarInfoSpec::TYPE_INT32) &&
        mds::Contains(types1, VarInfoSpec::TYPE_INT32)) {
      type_ = VarInfoSpec::TYPE_INT32;
    } else if (mds::Contains(types0, VarInfoSpec::TYPE_DOUBLE) &&
               mds::Contains(types1, VarInfoSpec::TYPE_DOUBLE)) {
      type_ = VarInfoSpec::TYPE_DOUBLE;
    } else if (mds::Contains(types0, VarInfoSpec::TYPE_BOOL) &&
               mds::Contains(types1, VarInfoSpec::TYPE_BOOL)) {
      if (op_ != '=' && op_ != '<') { // HACK
        return false;
      }
      type_ = VarInfoSpec::TYPE_BOOL;
    } else {
      return false;
    }
  }

  if (types) {
    if (op_ == '=' || op_ == '<') {  // TODO: extend to other types of booleans
      types->insert(VarInfoSpec::TYPE_BOOL);
    } else {
      types->insert(type_);
      if (type_ == VarInfoSpec::TYPE_INT32) {
        types->insert(VarInfoSpec::TYPE_DOUBLE);
      }
    }
  }
  return true;
}

void ArithNode::print(std::ostream& out,
                      const vector<VarHandle>* handles) const {
  out << "(";
  children_[0]->print(out, handles);
  out << " " << op_ << " ";
  children_[1]->print(out, handles);
  out << ")";
}

// IntConstNode ----------------------------------------------------------------
bool IntConstNode::TypeChecks(set<VarInfoSpec::VarType>* types) {
  if (types) {
    types->insert(VarInfoSpec::TYPE_INT32);
    types->insert(VarInfoSpec::TYPE_DOUBLE);
  }
  return true;
}

Gecode::LinIntExpr IntConstNode::GetIntExpr(
    TemplateModelHelper* helper) const {
  return Gecode::LinIntExpr(val_);
}

Gecode::LinFloatExpr IntConstNode::GetDoubleExpr(
    TemplateModelHelper* helper) const {
  return Gecode::LinFloatExpr(static_cast<double>(val_));
}

// DoubleConstNode -------------------------------------------------------------
bool DoubleConstNode::TypeChecks(set<VarInfoSpec::VarType>* types) {
  if (types) {
    types->insert(VarInfoSpec::TYPE_DOUBLE);
  }
  return true;
}

Gecode::LinFloatExpr DoubleConstNode::GetDoubleExpr(
    TemplateModelHelper* helper) const {
  return Gecode::LinFloatExpr(val_);
}

// BoolConstNode ---------------------------------------------------------------
bool BoolConstNode::TypeChecks(set<VarInfoSpec::VarType>* types) {
  if (types) {
    types->insert(VarInfoSpec::TYPE_BOOL);
  }
  return true;
}

}  // namespace mit_plato
