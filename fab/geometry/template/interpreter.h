#ifndef _FAB_GEOMETRY_TEMPLATE_INTERPETER_H__
#define _FAB_GEOMETRY_TEMPLATE_INTERPETER_H__

#include <string>
using std::string;

#include <boost/smart_ptr/scoped_ptr.hpp>
#include <boost/smart_ptr/shared_ptr.hpp>

#include "fab/geometry/template/editor.h"
#include "fab/geometry/template/params.h"

namespace mit_plato {

class FabGeometryTemplate;
class TemplateEditor;

//! Textual interpreter for issuing commands to the template
//! editor and for editing the tempalte from the commandline.
//!
//! Syntax and supported commands:
class PlatoInterpreter {
 public:

  //!
  //! Takes ownership of template object, must be non-NULL.
  PlatoInterpreter(boost::shared_ptr<FabGeometryTemplate> tpl,
                   const string& default_filename = "");

  void CoutCommands() const;

  bool GetYesNoAnswerCmd() const;

  bool ExecuteTopLevelCmd(const string& command);

  // SAVING --------------------------------------------------------------------
  void SaveTemplateCmd();

  void SaveModelCmd();

  // DISPLAY -------------------------------------------------------------------
  void PrintTreesCmd() const;

  void PrintParametersCmd() const;

  void PrintConstraintsCmd() const;

  void GetValueCmd() const;

  void SetValueCmd() const;

  // EDITING -------------------------------------------------------------------
  void CreateNodeCmd();

  void ModifyNodeCmd();

  void CombineNodesCmd();

  void InsertChildNodeCmd();

  void DeleteNodeCmd();

  void AddMetaParameterCmd();

  void DeleteMetaParameterCmd();

  void ChangeMutabilityCmd();

  void AddConstraintCmd();

  // UTILS ---------------------------------------------------------------------
  bool IsQuitCmd(const string& command) const;

  bool GetNonCancelInputCmd(string* command) const;

  // NON-COMMANDLINE METHODS ---------------------------------------------------

  bool SaveTemplate(const string& filename);

  bool SaveModel(const string& filename);

 private:

  bool PrintVariableValue(const VarInfo& info) const;

  void PrintOperationVariables(const OperationNode* op) const;

  void PrintDetailedParameters(const VarInfo& info,
                               const string& prefix = "") const;

  void PrintStatus(bool success) const;

  boost::scoped_ptr<mit_plato::TemplateEditor> editor_;
  string default_filename_;
};

}  // namespace mit_plato

#endif  // _FAB_GEOMETRY_TEMPLATE_INTERPETER_H__
