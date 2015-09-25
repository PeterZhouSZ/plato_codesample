#ifndef _FAB_GEOMETRY_TEMPLATE_OPERATIONS__SPECIAL_OPERATIONS_H__
#define _FAB_GEOMETRY_TEMPLATE_OPERATIONS__SPECIAL_OPERATIONS_H__

#include "fab/geometry/template/operations.h"

#include <vector>
#include <utility>
#include <string>
using std::vector;
using std::string;

#include "fab/geometry/template/operations.pb.h"
#include "fab/geometry/template/params.h"
#include "fab/geometry/template/template_params.h"

namespace mit_plato {

class ExprNode;

class IfElseIfOperation : public Operation {
 public:
  IfElseIfOperation();
  IfElseIfOperation(const IfElseIfOpSpec& spec);

  virtual bool GetCase(const NamedVarContainer& container,
                       int* matched_case);

  virtual OpType type() const { return IF_ELSE_IF; }

  virtual bool is_dirty() const {
    return handles_.size() > 0 || Operation::is_dirty() ; // HACK
  }
 private:
  vector<VarHandle> handles_;
  IfElseIfOpSpec spec_;
};


class ForOperation : public Operation {
 public:
  ForOperation();
  ForOperation(const ForOpSpec& spec);
  ~ForOperation() { CleanUp(); }

  bool Init(TemplateParams* container);

  bool Loop();

  void CleanUp();

  virtual OpType type() const { return FOR; }

  virtual bool is_dirty() const {
    return spec_.used_var_size() > 0 || Operation::is_dirty() ; // HACK
  }

 private:
  // These are reset every time Init is called
  int i_;
  TemplateParams* container_;
  vector<ExprNode*> exps_;

  vector<VarHandle> handles_;
  vector<std::pair<VarHandle, string> > assignments_;
  ForOpSpec spec_;
};

}  // namespace mit_plato

#endif
