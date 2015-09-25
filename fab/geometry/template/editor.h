#ifndef _FAB_GEOMETRY_TEMPLATE_EDITOR_H__
#define _FAB_GEOMETRY_TEMPLATE_EDITOR_H__

#include <string>
#include <set>
#include <vector>
using std::vector;

#include <boost/smart_ptr/shared_ptr.hpp>
#include "fab/geometry/template/constraints.h"
#include "fab/geometry/template/params.h"
#include "fab/geometry/template/template.h"

namespace mit_plato {

class FabGeometryTemplate;
class OperationNode;
class Constraint;

//! Utility for editing the geometry template. Intended for the
//! professional user (e.g. via GUI).
class TemplateEditor {
 public:
  typedef ConstraintParser<TemplateParams> ConParser;
  typedef ExpressionParser<TemplateParams> ExprParser;

  TemplateEditor(boost::shared_ptr<FabGeometryTemplate> tpl);
  ~TemplateEditor() {}

  boost::shared_ptr<const FabGeometryTemplate> tpl() const { return tpl_; }
  boost::shared_ptr<FabGeometryTemplate> tpl() { return tpl_; }

  bool CreateShape(const string& registered_name);

  bool ModifyShape(const string& registered_name);

  bool CombineShapes(const string& registered_name);

  bool DeleteShapes();

  bool AddConstraint(const string& expression,
                     const vector<VarHandle>& vars);

  bool DeleteConstraint(const int index);

  // Takes ownership
  bool AddConstraint(Constraint* con);

  // Takes ownership; replaces constraint at index with a new set of constraints
  bool ReplaceConstraint(const int index,
                         const vector<Constraint*>& constraints);

  void AddDefaultGlobalProperties();

  // Selection
  void ClearSelection();
  bool SetSelected(const string& node_name);
  void SetSelected(const OperationNode* node);
  void ToggleSelected(const OperationNode* node);

  bool IsSelected(const OperationNode* node) const;
  const std::vector<const OperationNode*>& selected_nodes() const;
  const std::set<const Constraint*>& selected_constraints() const;

  // Mutable parameters
  TemplateParams& params() { return tpl_->params_; }


  // Lower level methods
  bool AddNode(OperationNode* node);

 private:
  TemplateEditor() {}

  // Deletes node in a smart fashion, leaving a valid operations tree;
  // may delete whole subtrees if they are no longer valid.
  bool DeleteNode(OperationNode* node);

  // Deletes only one node and updates parent/child references
  // Note: may result in invalid operations tree, call with caution!
  void CollapseNode(OperationNode* node);

  // Deletes the part of the tree originating in this top_node,
  // updates parent references and removes the direct parent if it is
  // a combine node left with less than 2 children.
  // Note: may result in invalid operations tree, call with caution!
  void DeleteBranch(OperationNode* top_node);

  // Selection
  std::vector<const OperationNode*> selected_;
  std::set<const Constraint*> selected_cons_;

  boost::shared_ptr<FabGeometryTemplate> tpl_;
};

}  // namespace mit_plato

#endif  // _FAB_GEOMETRY_TEMPLATE_EDITOR_H__
