#ifndef _FAB_GEOMETRY_TEMPLATE_CONSTRAINTS_EXPRESSION_H__
#define _FAB_GEOMETRY_TEMPLATE_CONSTRAINTS_EXPRESSION_H__

#include <cctype>
#include <cstdlib>
#include <iostream>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
using std::runtime_error;
using std::set;
using std::string;
using std::stringstream;
using std::vector;

#include "util/stl_util.h"
#include <gecode/float.hh>
#include <gecode/int.hh>
#include <gecode/minimodel.hh>
#include <glog/logging.h>
#include <google/protobuf/stubs/common.h>
using google::protobuf::int32;

#include "fab/geometry/template/constraints/model_helper.h"
#include "fab/geometry/template/operations.pb.h"
#include "fab/geometry/template/params.h"
#include "fab/geometry/template/template_params.h"


namespace mit_plato {

class ExprNode {
 public:
  ExprNode() {}

  // Takes ownership of children
  ExprNode(const vector<ExprNode*>* children);

  virtual ~ExprNode();

  // Base implementation throws runtime_error
  virtual int32 EvaluateInt() const;

  // Base implementation throws runtime_error
  virtual double EvaluateDouble() const;

  // Base implementation throws runtime_error
  virtual bool EvaluateBool() const;


  virtual Gecode::LinIntExpr GetIntExpr(TemplateModelHelper* helper) const;

  virtual Gecode::LinFloatExpr GetDoubleExpr(TemplateModelHelper* helper) const;

  virtual Gecode::BoolExpr GetBoolExpr(TemplateModelHelper* helper) const;


  // Returns true if the expression type checks
  virtual bool TypeChecks(set<VarInfoSpec::VarType>* types = NULL) = 0;

  // Prints expression and its subexpressions;
  // if handles are set, subs all actual var names with $arg_num
  virtual void print(std::ostream& out,
                     const vector<VarHandle>* handles = NULL) const = 0;

  // Convenience method for print
  string ToString() const;

  // Returns all var handles involved in this expression
  virtual void GetUsedHandles(set<VarHandle>* handles) const;

 protected:
  vector<ExprNode*> children_;
};

inline std::ostream& operator<<(std::ostream& os,
                                const ExprNode& expr) {
  expr.print(os);
  return os;
}

class UnaryFunctionNode : public ExprNode {
 public:
  UnaryFunctionNode(const string& op,
                    ExprNode* child);

  // TODO: may want to also support int expressions

  virtual Gecode::LinFloatExpr GetDoubleExpr(TemplateModelHelper* helper) const;

  virtual double EvaluateDouble() const;

  virtual bool TypeChecks(set<VarInfoSpec::VarType>* types = NULL);

  virtual void print(std::ostream& out,
                     const vector<VarHandle>* handles = NULL) const;
 private:
  void CheckSanity() const;
  const string op_;
};


// Times(*), plus(+), minus(-), divide(/)
// TODO: extend with dot and cross for vectors, as well as
// scalar / vector operations
class ArithNode : public ExprNode {
 public:
  ArithNode(const char op,
            ExprNode* child1,
            ExprNode* child2);

  virtual Gecode::LinIntExpr GetIntExpr(TemplateModelHelper* helper) const;

  virtual Gecode::LinFloatExpr GetDoubleExpr(TemplateModelHelper* helper) const;

  virtual int32 EvaluateInt() const;

  virtual double EvaluateDouble() const;

  virtual bool EvaluateBool() const;

  virtual bool TypeChecks(set<VarInfoSpec::VarType>* types = NULL);

  virtual void print(std::ostream& out,
                     const vector<VarHandle>* handles = NULL) const;

 private:
  void CheckSanity() const;

  template <class A, class B, class C>
  bool Evaluate(const A& v1, const B& v2, C* res) const {
    switch (op_) {
      case '*':
        *res = v1 * v2;
        return true;
      case '/':
        *res = v1 / v2;
        return true;
      case '+':
        *res = v1 + v2;
        return true;
      case '-':
        *res = v1 - v2;
        return true;
      default:
        return false;
    }
  }

  const char op_;
  VarInfoSpec::VarType type_;
};

template <class Container>
class RefNode : public ExprNode {
 public:
  RefNode(const VarHandle& handle,
          const Container* container) : handle_(handle),
                                        container_(CHECK_NOTNULL(container)) {}

  virtual void print(std::ostream& out,
                     const vector<VarHandle>* handles = NULL) const;

  virtual void GetUsedHandles(set<VarHandle>* handles) const {
    handles->insert(handle_);
  }

 protected:
  RefNode() {}

  const VarHandle handle_;
  const Container* container_;
};


template <class Container>
class IntRefNode : public RefNode<Container> {
 public:
  IntRefNode(const VarHandle& handle,
             const Container* container);

  virtual Gecode::LinIntExpr GetIntExpr(TemplateModelHelper* helper) const;

  virtual Gecode::LinFloatExpr GetDoubleExpr(TemplateModelHelper* helper) const;

  virtual int32 EvaluateInt() const {
    // Alas, "this" ugliness is necessary
    return this->container_->template Get<int32>(this->handle_);
  }

  virtual double EvaluateDouble() const {
    return this->container_->template Get<int32>(this->handle_);
  }

  virtual bool TypeChecks(set<VarInfoSpec::VarType>* types = NULL);
};


template <class Container>
class DoubleRefNode : public RefNode<Container> {
 public:
  DoubleRefNode(const VarHandle& handle,
                const Container* container);

  virtual Gecode::LinFloatExpr GetDoubleExpr(TemplateModelHelper* helper) const;

  virtual double EvaluateDouble() const {
    return this->container_->template Get<double>(this->handle_);
  }

  virtual bool TypeChecks(set<VarInfoSpec::VarType>* types = NULL);
};


template <class Container>
class BoolRefNode : public RefNode<Container> {
 public:
  BoolRefNode(const VarHandle& handle,
              const Container* container);

  //  virtual Gecode::BoolExpr GetBoolExpr(TemplateModelHelper* helper) const;

  virtual bool EvaluateBool() const {
    return this->container_->template Get<bool>(this->handle_);
  }

  virtual bool TypeChecks(set<VarInfoSpec::VarType>* types = NULL);
};


class IntConstNode : public ExprNode {
 public:
  IntConstNode(int32 val) : val_(val) {}

  virtual Gecode::LinIntExpr GetIntExpr(TemplateModelHelper* helper) const;

  virtual Gecode::LinFloatExpr GetDoubleExpr(TemplateModelHelper* helper) const;

  virtual int32 EvaluateInt() const { return val_; }

  virtual double EvaluateDouble() const { return val_; }

  virtual void print(std::ostream& out,
                     const vector<VarHandle>* handles = NULL) const {
    out << val_;
  }

  virtual bool TypeChecks(set<VarInfoSpec::VarType>* types = NULL);

 private:
  IntConstNode() {}
  int32 val_;
};


class DoubleConstNode : public ExprNode {
 public:
  DoubleConstNode(double val) : val_(val) {}

  virtual Gecode::LinFloatExpr GetDoubleExpr(TemplateModelHelper* helper) const;

  virtual double EvaluateDouble() const { return val_; }

  virtual void print(std::ostream& out,
                     const vector<VarHandle>* handles = NULL) const {
    out << val_;
  }

  virtual bool TypeChecks(std::set<VarInfoSpec::VarType>* types = NULL);

 private:
  DoubleConstNode() {}
  double val_;
};


class BoolConstNode : public ExprNode {
 public:
  BoolConstNode(bool val) : val_(val) {}

  virtual bool EvaluateBool() const { return val_; }

  virtual void print(std::ostream& out,
                     const vector<VarHandle>* handles = NULL) const {
    out << (val_ ? "true" : "false");
  }

  virtual bool TypeChecks(std::set<VarInfoSpec::VarType>* types = NULL);

 private:
  BoolConstNode() {}
  bool val_;
};


// Parser ----------------------------------------------------------------------

template <class Container>
class ExpressionParser {
 public:
  ExpressionParser(const vector<VarHandle>& handles,
                   const Container* container,
                   std::istream& expr);

  ExprNode* ParseUnsafe();

  ExprNode* Parse(string* error_msg = NULL);

 protected:
  struct Token {
    static const char INVALID_TYPE;
    static const char NUM_TYPE;
    static const char ALPHA_TYPE;
    static const char REF_TYPE;

    Token() : type(Token::INVALID_TYPE) {}
    Token(char t, const string& c = "") : type(t), content(c) {
      if (content.empty() && t != Token::INVALID_TYPE) {
        content = string(1, type);
      }
    }

    void putback(std::istream& expr) {
      if (!expr) {
        expr.clear();
      }
      if (type == REF_TYPE) expr.putback('$');
      for (int i = content.size() - 1; i >= 0; --i) {
        expr.putback(content[i]);
      }
    }

    char type;
    string content;
  };

  Token GetToken();
  ExprNode* GetExpression();
  ExprNode* GetTerm();
  ExprNode* GetPrimary(bool negate_number = false);
  ExprNode* CreateRefNode(int arg_num) const;

  vector<VarHandle> handles_;
  const Container* container_;
  std::istream& expr_;
};


// -----------------------------------------------------------------------------
// DEFINITIONS
// -----------------------------------------------------------------------------

// RefNode ------------------------------------------------------------------

template <class Container>
void RefNode<Container>::print(std::ostream& out,
                               const vector<VarHandle>* handles) const {
  if (!handles) {
    out << handle_;
    return;
  }

  for (int i = 0; i < handles->size(); ++i) {
    if ((*handles)[i] == handle_) {
      out << "$" << i;
      return;
    }
  }
  throw runtime_error("Failed to sub handle with arguments");
}

// IntRefNode ------------------------------------------------------------------

template <class Container>
IntRefNode<Container>::IntRefNode(const VarHandle& handle,
                                  const Container* container)
    : RefNode<Container>(handle, container) {
  CHECK_EQ(VarInfoSpec::TYPE_INT32,
           CHECK_NOTNULL(this->container_->GetVarInfo(handle))->type());
}

template <class Container>
bool IntRefNode<Container>::TypeChecks(set<VarInfoSpec::VarType>* types) {
  if (types) {
    types->insert(VarInfoSpec::TYPE_INT32);
    types->insert(VarInfoSpec::TYPE_DOUBLE);
  }
  return true;
}

template <class Container>
Gecode::LinIntExpr IntRefNode<Container>::GetIntExpr(
    TemplateModelHelper* helper) const {
  if (helper->HasVar(this->handle_)) {
    return Gecode::LinIntExpr(helper->GetVarInt(this->handle_));
  } else {
    return Gecode::LinIntExpr(EvaluateInt());
  }
}

template <class Container>
Gecode::LinFloatExpr IntRefNode<Container>::GetDoubleExpr(
    TemplateModelHelper* helper) const {
  if (!helper->HasVar(this->handle_)) {
    return Gecode::LinFloatExpr(EvaluateDouble());
  } else {
    // Need to channel
    VarHandle handle = helper->AuxDoubleHandle(this->ToString());
    if (helper->HasVar(handle)) {
      return Gecode::LinFloatExpr(helper->GetVarDouble(handle));
    } else {
      const Gecode::IntVar& int_var = helper->GetVarInt(this->handle_);
      const Gecode::FloatVar& float_var =
          helper->NewAuxDouble(handle, int_var.min(), int_var.max());
      Gecode::channel(helper->home(), float_var, int_var);
      return Gecode::LinFloatExpr(float_var);
    }
  }
}

// DoubleRefNode ---------------------------------------------------------------

template <class Container>
DoubleRefNode<Container>::DoubleRefNode(const VarHandle& handle,
                                        const Container* container)
    : RefNode<Container>(handle, container) {
  CHECK_EQ(VarInfoSpec::TYPE_DOUBLE,
           CHECK_NOTNULL(this->container_->GetVarInfo(handle))->type());
}

template <class Container>
bool DoubleRefNode<Container>::TypeChecks(set<VarInfoSpec::VarType>* types) {
  if (types) {
    types->insert(VarInfoSpec::TYPE_DOUBLE);
  }
  return true;
}

template <class Container>
Gecode::LinFloatExpr DoubleRefNode<Container>::GetDoubleExpr(
    TemplateModelHelper* helper) const {
  if (helper->HasVar(this->handle_)) {
    return Gecode::LinFloatExpr(helper->GetVarDouble(this->handle_));
  } else {
    return Gecode::LinFloatExpr(EvaluateDouble());
  }
}

// BoolRefNode -----------------------------------------------------------------

template <class Container>
BoolRefNode<Container>::BoolRefNode(const VarHandle& handle,
                                    const Container* container)
    : RefNode<Container>(handle, container) {
  CHECK_EQ(VarInfoSpec::TYPE_BOOL,
           CHECK_NOTNULL(this->container_->GetVarInfo(handle))->type());
}

template <class Container>
bool BoolRefNode<Container>::TypeChecks(set<VarInfoSpec::VarType>* types) {
  if (types) {
    types->insert(VarInfoSpec::TYPE_BOOL);
  }
  return true;
}

// -----------------------------------------------------------------------------
// PARSER
// -----------------------------------------------------------------------------
template <class Container>
const char ExpressionParser<Container>::Token::INVALID_TYPE = '0';

template <class Container>
const char ExpressionParser<Container>::Token::NUM_TYPE = '1';

template <class Container>
const char ExpressionParser<Container>::Token::ALPHA_TYPE = 'a';

template <class Container>
const char ExpressionParser<Container>::Token::REF_TYPE = 'r';

template <class Container>
ExpressionParser<Container>::ExpressionParser(const vector<VarHandle>& handles,
                                              const Container* container,
                                              std::istream& expr)
    : handles_(handles),
      container_(container),
      expr_(expr) {}

template <class Container>
ExprNode* ExpressionParser<Container>::ParseUnsafe() {
  ExprNode* node = GetExpression();

  Token t = GetToken();
  VLOG(5) << "Token content: " << t.content;
  if (!t.content.empty()) {
    delete node;
    throw runtime_error(
        "End of expression expected before token " + t.content);
  }
  return node;
}

template <class Container>
ExprNode* ExpressionParser<Container>::Parse(string* error_msg) {
  try {
    return ParseUnsafe();
  } catch (runtime_error& e) {
    LOG(ERROR) << e.what();
    if (error_msg) {
      *error_msg = e.what();
    }
    return NULL;
  }
}

template <class Container>
ExprNode* ExpressionParser<Container>::CreateRefNode(int arg_num) const {
  if (arg_num < 0 || arg_num >= handles_.size()) {
    stringstream ss;
    ss << "Out of bounds argument: $" << arg_num;
    throw runtime_error(ss.str());
  }

  const VarHandle& handle = handles_[arg_num];
  const VarInfo* info = container_->GetVarInfo(handle);
  switch (info->type()) {
    case VarInfoSpec::TYPE_INT32:
      return new IntRefNode<Container>(handle, container_);
    case VarInfoSpec::TYPE_DOUBLE:
      return new DoubleRefNode<Container>(handle, container_);
    case VarInfoSpec::TYPE_BOOL:
      return new BoolRefNode<Container>(handle, container_);
    default:
      {
        stringstream ss;
        ss << "Argument referencing not implemented for type: "
           << VarInfoSpec::VarType_Name(info->type());
        throw runtime_error(ss.str());
        return NULL;
      }
  }
}

template <class Container>
ExprNode* ExpressionParser<Container>::GetPrimary(bool negate_number) {
  Token t = GetToken();
  switch (t.type) {
    case '(':    // handle '(' expression ')'
      if (negate_number) {
        throw runtime_error(
            "Negating arbitrary terms using '-' is not supported. "
            "Try multiplying by -1 instead.");
      } else {
        ExprNode* e = GetExpression();
        t = GetToken();
        if (t.type != ')') {
          delete e;
          throw runtime_error("Unbalanced parentheses; expected ')'.");
        }
        return e;
      }
    case '-':
      return GetPrimary(!negate_number);
    case Token::REF_TYPE:
      {
        int val = atoi(t.content.c_str());
        // TODO: check for errors
        return CreateRefNode(val);
      }
    case Token::NUM_TYPE:
      {
        bool has_dot = t.content.find('.') != string::npos;
        stringstream ss(t.content);
        if (has_dot) {
          double val;
          ss >> val;
          if (negate_number) val *= -1.0;
          return new DoubleConstNode(val);
        } else {
          int32 val;
          ss >> val;
          if (negate_number) val *= -1.0;
          return new IntConstNode(val);
        }
      }
    case Token::ALPHA_TYPE:
      if (t.content == "true") {
        return new BoolConstNode(true);
      } else if (t.content == "false") {
        return new BoolConstNode(false);
      } else {
        Token t2 = GetToken();
        if (t2.type != '(') {
          throw runtime_error("Expected '(' after function " + t.content);
        }
        ExprNode* e = GetExpression();
        t2 = GetToken();
        if (t2.type != ')') {
          delete e;
          throw runtime_error("Unbalanced parentheses; expected ')'.");
        }
        return new UnaryFunctionNode(t.content, e);
        // TODO: add support for non-unary functions
      }
    default:
      throw runtime_error("Unexpected token: " + t.content);
  }
}

template <class Container>
ExprNode* ExpressionParser<Container>::GetTerm() {
  ExprNode* left = GetPrimary();

  Token t = GetToken();
  while (true) {
    switch (t.type) {
      case '*':
      case '/':
      case '=':
      case '<':
        try {
          ExprNode* right = GetPrimary();
          left = new ArithNode(t.type, left, right);
          t = GetToken();
        } catch (runtime_error& e) {
          delete left;
          throw e;
	}
        break;
      default:
        t.putback(expr_);
        return left;
    }
  }
}

template <class Container>
ExprNode* ExpressionParser<Container>::GetExpression() {
  ExprNode* left = GetTerm();

  Token t = GetToken();
  while(true) {
    switch(t.type) {
      case '+':
      case '-':
        try {
          ExprNode* right = GetTerm();
          left = new ArithNode(t.type, left, right);
          t = GetToken();
        } catch (runtime_error& e) {
          delete left;
          throw e;
	}
        break;
      default:
        t.putback(expr_);
        return left;
    }
  }
}

template <class Container>
typename ExpressionParser<Container>::Token ExpressionParser<Container>::GetToken() {
  char ch = 0;
  if (!(expr_ >> ch)) {
    return Token();
  }

  bool has_dot = false;
  switch (ch) {
    case '(':
    case ')':
    case '+':
    case '*':
    case '/':
    case '-':
    case ',':
    case ';':
    case '=':
    case '<':
      return Token(ch);
    case '$':
      {
        string s;
        while (expr_.get(ch)) {
          if (isdigit(ch)) {
            s += ch;
          } else {
            expr_.putback(ch);
            break;
          }
        }
        return Token(Token::REF_TYPE, s);
      }
    case '.':
      has_dot = true;
    case '0': case '1': case '2': case '3': case '4':
    case '5': case '6': case '7': case '8': case '9':
      {
        string s;
        s += ch;
        while (expr_.get(ch)) {
          if (isdigit(ch)) {
            s += ch;
          } else if (ch == '.') {
            if (has_dot) {
              LOG(ERROR) << "Two dots in numeral, while parsing: " << s;
              return Token(Token::INVALID_TYPE, s + ".");
            }
            has_dot = true;
            s+= ch;
          } else {
            expr_.putback(ch);
            break;
          }
        }
        return Token(Token::NUM_TYPE, s);
      }
    default:
      if (isalpha(ch)) {
        string s;
        s += ch;
        while (expr_.get(ch)) {
          if (isalpha(ch) || ch == '_') {
            s+= ch;
          } else {
            expr_.putback(ch);
            break;
          }
        }
        return Token(Token::ALPHA_TYPE, s);
      } else {
        return Token(Token::INVALID_TYPE, string(1, ch));
      }
  }
}

}  // namespace mit_plato

#endif  // _FAB_GEOMETRY_TEMPLATE_CONSTRAINTS_EXPRESSION_H__
