#include "fab/geometry/template/interpreter.h"

#include <iostream>
using std::cin;
using std::cout;
using std::endl;

#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <google/protobuf/message.h>
#include <google/protobuf/stubs/common.h>

#include "fab/geometry/template/library.h"
#include "fab/geometry/template/operations.pb.h"
#include "fab/geometry/template/params.h"
#include "fab/geometry/template/shape.h"
#include "fab/geometry/template/template.h"
#include "fab/geometry/tri_mesh.h"
#include "util/proto_util.h"

namespace mit_plato {

using google::protobuf::Message;
using google::protobuf::int32;

PlatoInterpreter::PlatoInterpreter(boost::shared_ptr<FabGeometryTemplate> tpl,
                                   const string& default_filename)
    : editor_(new TemplateEditor(tpl)),
      default_filename_(default_filename) {}

bool PlatoInterpreter::GetYesNoAnswerCmd() const {
  string answer;
  std::getline(cin, answer);

  if (answer == "y" || answer == "Y") {
    return true;
  } else if (answer == "n" || answer == "N") {
    return false;
  } else {
    cout << "   Unknown option \"" << answer
         << "\". Please enter \"y\" or \"n\": ";
    return GetYesNoAnswerCmd();
  }
}

void PlatoInterpreter::CoutCommands() const {
  cout << " BASIC COMMANDS -------------------------------" << endl;
  cout << " q  - quit (or cancell current command)" << endl;
  cout << " s  - save current template to ASCII file" << endl;
  cout << " sm - save current mesh to file" << endl;
  cout << " p  - print operation tree(s)" << endl;

  cout << " EDIT COMMANDS --------------------------------" << endl;
  cout << " cre - add a Create node" << endl;
  cout << " mod - add a Modify node" << endl;
  cout << " com - combine two or more nodes" << endl;
  cout << " ins - insert child subtree into existing combine node" << endl;
  cout << " del - delete a node" << endl;

  cout << " CONSTRAINT COMMANDS ---------------------------" << endl;
  cout << " pc - print constraints" << endl;
  cout << " ac - add constraint" << endl;

  cout << " PARAMETER COMMANDS ----------------------------" << endl;
  cout << " pp - print parameters" << endl;
  cout << " mut - mark certain params as mutable / immutable " << endl;
  cout << " cmeta - create meta parameter" << endl;
  cout << " dmeta - delete meta parameter" << endl;
  cout << " get - gets value and information about param / property " << endl;
  cout << " set - set value of a param" << endl;
}

bool PlatoInterpreter::ExecuteTopLevelCmd(const string& command) {
  if (command == "s" || command == "S") {
    cout << "--> Preparing to save template text file." << endl;
    SaveTemplateCmd();
  } else if (command == "sm" || command == "SM") {
    cout << "--> Preparing to save mesh to file." << endl;
    SaveModelCmd();
  } else if (command == "p" || command == "P") {
    cout << "--> Preparing to print operation tree(s)." << endl;
    PrintTreesCmd();
  } else if (command == "pp" || command == "PP") {
    cout << "--> Preparing to print parameters." << endl;
    PrintParametersCmd();
  } else if (command == "mut") {
    cout << "--> Preparing to change mutability." << endl;
    ChangeMutabilityCmd();
  } else if (command == "cmeta") {
    cout << "--> Preparing to add meta-parameter." << endl;
    AddMetaParameterCmd();
  } else if (command == "dmeta") {
    cout << "--> Preparing to delete meta-parameter." << endl;
    DeleteMetaParameterCmd();
  } else if (command == "get") {
    cout << "--> Preparing to get parameter / property info." << endl;
    GetValueCmd();
  } else if (command == "set") {
    cout << "--> Preparing to set parameter value." << endl;
    SetValueCmd();
  } else if (command == "cre") {
    cout << "--> Preparing to add a Create Geometry node" << endl;
    CreateNodeCmd();
  } else if (command == "mod") {
    cout << "--> Preparing to add a Modify Geometry node" << endl;
    ModifyNodeCmd();
  } else if (command == "com") {
    cout << "--> Preparing to add a Combine Geometries node" << endl;
    CombineNodesCmd();
  } else if (command == "ins") {
    cout << "--> Preparing to add a child to an existing Combine node" << endl;
    InsertChildNodeCmd();
  } else if (command == "del") {
    cout << "--> Preparing to delete a node" << endl;
    DeleteNodeCmd();
  } else if (command == "pc") {
    cout << "--> Preparing to print constraints" << endl;
    PrintConstraintsCmd();
  } else if (command == "ac") {
    cout << "--> Preparing to add constraint" << endl;
    AddConstraintCmd();
  } else {
    cout << "   Unknown command \"" << command << "\"";
    return false;
  }
  return true;
}

bool PlatoInterpreter::IsQuitCmd(const string& command) const {
  return command == "q" || command == "Q";
}

bool PlatoInterpreter::GetNonCancelInputCmd(string* command) const {
  cin.clear();
  std::getline(cin, *command);

  if (!IsQuitCmd(*command)) {
    return true;
  }

  cout << "  --> Cancelled" << endl;
  return false;
}

void PlatoInterpreter::SaveTemplateCmd() {
  cout << "  > Enter filename [ " << default_filename_ << " ]: ";

  string tmp_filename;
  if (!GetNonCancelInputCmd(&tmp_filename)) { return; }

  if (!tmp_filename.empty()) {
    default_filename_ = tmp_filename;
  }

  if (SaveTemplate(default_filename_)) {
    cout << "  --> Saved successfully" << endl;
  } else {
    cout << "  --> Failed to save. Did you enter a valid filename?" << endl;
    SaveTemplateCmd();
  }
}

void PlatoInterpreter::SaveModelCmd() {
  cout << "  > Enter filename [  ]: ";

  string tmp_filename;
  if (!GetNonCancelInputCmd(&tmp_filename)) { return; }

  if (SaveModel(tmp_filename)) {
    cout << "  --> Saved successfully" << endl;
  } else {
    cout << "  --> Failed to save. Did you enter a valid filename?" << endl;
    SaveModelCmd();
  }
}

void PlatoInterpreter::PrintStatus(bool success) const {
  if (success) {
    cout << "  --> Successful" << endl;
  } else {
    cout << "  --> Failed" << endl;
  }
}

void PlatoInterpreter::CreateNodeCmd() {
  cout << "  > Available Create Nodes: " << endl;
  for (const string& name : FabTemplateLibrary::RegisteredShapes()) {
    cout << "    > " << name << endl;
  }
  cout << "  > Enter choice: ";

  string input;
  if (!GetNonCancelInputCmd(&input)) { return; }

  PrintStatus(editor_->CreateShape(input));
  editor_->ClearSelection();
}

void PlatoInterpreter::DeleteNodeCmd() {
  PrintTreesCmd();
  cout << "  > Enter node to delete: ";

  string input;
  if (!GetNonCancelInputCmd(&input)) { return; }
  editor_->SetSelected(input);

  PrintStatus(editor_->DeleteShapes());
  editor_->ClearSelection();
}

void PlatoInterpreter::ModifyNodeCmd() {
  PrintTreesCmd();
  cout << "  > Enter node to modify: ";

  {
    string input;
    if (!GetNonCancelInputCmd(&input)) { return; }
    editor_->SetSelected(input);
  }

  cout << "  > Available Modify Nodes: " << endl;
  for (const string& name : FabTemplateLibrary::RegisteredTransformations()) {
    cout << "    > " << name << endl;
  }
  cout << "  > Enter choice: ";

  string input;
  if (!GetNonCancelInputCmd(&input)) { return; }

  PrintStatus(editor_->ModifyShape(input));
  editor_->ClearSelection();
}

void PlatoInterpreter::InsertChildNodeCmd() {
  cout << "OOPS, not implemented yet!" << endl;
  //PrintTreesCmd();
  //cout << "  > Enter existing combine node to add to: ";
}

void PlatoInterpreter::CombineNodesCmd() {
  PrintTreesCmd();
  cout << "  > Enter 2 or more nodes to combine (separate with a comma): ";

  vector<string> node_names;
  {
    string input;
    if (!GetNonCancelInputCmd(&input)) { return; }
    boost::split(node_names, input, boost::is_any_of(","));
  }

  if (node_names.size() < 2) {
    cout << "  --> Error: must enter at least 2 nodes" << endl;
    return;
  }

  for (const string& node_name : node_names) {
    if (!editor_->SetSelected(node_name)) {
      cout << "  --> Error: could not select \"" << node_name << "\"" << endl;
      editor_->ClearSelection();
      return;
    }
  }

  cout << "  > Available Combine Nodes: " << endl;
  for (const string& name : FabTemplateLibrary::RegisteredCombineOperations()) {
    cout << "    > " << name << endl;
  }
  cout << "  > Enter choice: ";

  string input;
  if (!GetNonCancelInputCmd(&input)) { return; }

  PrintStatus(editor_->CombineShapes(input));
  editor_->ClearSelection();
}

bool PlatoInterpreter::SaveTemplate(const string& filename) {
  FabTemplateProto tpl_proto;
  CHECK(editor_->tpl()->ToSpec(&tpl_proto));

  return mds_util::WriteASCIIProtoToFile(filename, tpl_proto);
}

bool PlatoInterpreter::SaveModel(const string& filename) {
  // First unite all meshes into one
  TriMesh combined;
  for (const auto& shape_ptr : editor_->tpl()->shapes()) {
    combined.AddData(shape_ptr->Mesh());
  }

  return OpenMesh::IO::write_mesh(combined, filename);
}

namespace {
int CountChars(const vector<const OperationNode*>& nodes) {
  int sum = 0;
  for (const OperationNode* node : nodes) {
    sum += node->name().size();
  }
  return sum;
}

// Returns the length of the appended string
int PrintNode(const OperationNode* node, string* out) {
  vector<const OperationNode*> leaves;
  node->GetLeaves(&leaves);
  int num_chars = CountChars(leaves) + 5 * leaves.size();

  size_t padding = (num_chars - node->name().size()) / 2.0;
  *out += string(padding, ' ') + node->name() + string(padding, ' ');
  return padding * 2 + node->name().size();
}

void PrintLines(const vector<const OperationNode*>& layer_nodes) {
  if (layer_nodes.empty()) return;

  string line1;
  string line2;

  bool real_children_exist = false;
  vector<const OperationNode*> next_layer;
  vector<OperationNode*> created_nodes;
  for (const OperationNode* node : layer_nodes) {
    int width = PrintNode(node, &line2);
    line1 += string(width / 2, ' ')
             + (node->operation() ? '|' : ' ')  // only non-dummy nodes have stems
             + string(width / 2, ' ');

    // Add next layer for printing
    for (const OperationNode* child : node->children()) {
      next_layer.push_back(child);
      real_children_exist = true;
    }
    // Add a dummy child so that the children of siblings are
    // appropriately displaced.
    if (node->children().empty()) {
      created_nodes.push_back(
          OperationNodeFactory::Singleton().Create(
              string(node->name().size(), ' '), true));
      next_layer.push_back(created_nodes.back());
    }
  }

  cout << line1 << endl;
  cout << line1 << endl;
  cout << line1 << endl;
  cout << line2 << endl;

  if (real_children_exist)
    PrintLines(next_layer);

  for (OperationNode* node : created_nodes) {
    delete node;
  }
}
}

void PlatoInterpreter::PrintTreesCmd() const {
  vector<const OperationNode*> nodes(
      editor_->tpl()->roots().begin(), editor_->tpl()->roots().end());
  PrintLines(nodes);
}

void PlatoInterpreter::PrintDetailedParameters(const VarInfo& info,
                                               const string& prefix) const {
  cout << prefix << info
       << (editor_->params().IsMutable(info.handle()) ? " - <<MUTABLE>>" : "")
       << endl;

  set<VarInfo> child_vars =
      editor_->tpl()->params_container().child_vars(info.handle());
  for (const VarInfo& child : child_vars) {
    PrintDetailedParameters(child, prefix + "  ");
  }
}

void PlatoInterpreter::PrintParametersCmd() const {
  cout << "  > Enter operation name (if none, prints all high-level params): ";

  string op_name;
  if (!GetNonCancelInputCmd(&op_name)) { return; }

  const TemplateParams& params = editor_->tpl()->params_container();
  for (const auto& var_info : params.vars()) {
    if (op_name.empty()) {
      cout << var_info
           << (params.IsMutable(var_info.handle()) ? " - <<MUTABLE>>" : "")
           << endl;
    } else if (op_name == var_info.name_space()) {
      PrintDetailedParameters(var_info);
    }
  }
}

void PlatoInterpreter::PrintConstraintsCmd() const {
  for (const Constraint* con : editor_->tpl()->constraints()) {
    cout << con->ShortDescription() << endl;
  }
}

void PlatoInterpreter::ChangeMutabilityCmd() {
  PrintTreesCmd();
  cout << "  > Enter operation name (leave blank for meta params): ";
  string name_space;
  if (!GetNonCancelInputCmd(&name_space)) { return; }

  cout << "  > Enter parameter name: ";
  string name;
  if (!GetNonCancelInputCmd(&name)) { return; }

  VarHandle handle(
      name_space, name,
      (name_space.empty() ? VarHandleSpec::META : VarHandleSpec::PARAM));
  if (editor_->params().IsMutable(handle)) {
    cout << "  --> Variable is mutable, setting to IMMUTABLE." << endl;
    PrintStatus(editor_->params().MarkAsImmutable(handle));
  } else {
    const VarInfo* info = editor_->params().GetVarInfo(handle);
    if (!info) {
      cout << "  --> Error: could not find variable " << handle << endl;
      return;
    }
    if (info->type() == VarInfoSpec::TYPE_BOOL) {
      PrintStatus(editor_->params().MarkAsMutableBool(handle));
    } else if (info->type() == VarInfoSpec::TYPE_INT32) {
      int min, max;
      cout << "  > Enter min value: ";
      cin >> min;
      cout << "  > Enter max value: ";
      cin >> max;
      PrintStatus(editor_->params().MarkAsMutableInt(
          handle, new IntBounds(min, max)));
    } else if (info->type() == VarInfoSpec::TYPE_DOUBLE) {
      double min, max;
      cout << "  > Enter min value: ";
      cin >> min;
      cout << "  > Enter max value: ";
      cin >> max;
      PrintStatus(editor_->params().MarkAsMutableDouble(
          handle, new DoubleBounds(min, max)));
    } else {
      cout << "  --> Error: not implemented for type "
           << VarInfoSpec::VarType_Name(info->type());
    }
  }
}

void PlatoInterpreter::AddMetaParameterCmd() {
  cout << "  > Available Types: " << endl;
  cout << "    > " << VarInfoSpec::VarType_Name(VarInfoSpec::TYPE_INT32) << endl;
  cout << "    > " << VarInfoSpec::VarType_Name(VarInfoSpec::TYPE_DOUBLE) << endl;
  cout << "    > " << VarInfoSpec::VarType_Name(VarInfoSpec::TYPE_BOOL) << endl;
  cout << "    > " << VarInfoSpec::VarType_Name(VarInfoSpec::TYPE_STRING) << endl;

  cout << "  > Enter choice: ";

  string input;
  if (!GetNonCancelInputCmd(&input)) { return; }

  VarInfoSpec::VarType type = VarInfoSpec::TYPE_UNK;
  if (!VarInfoSpec::VarType_Parse(input, &type)) {
    cout << "  --> Error: invalid type " << input << endl;
  } else {
    cout << "  > Enter variable name: ";

    string name;
    if (!GetNonCancelInputCmd(&name)) { return; }
    VarHandle handle;
    PrintStatus(editor_->params().CreateMetaVariable(name, type, &handle));
  }
}

void PlatoInterpreter::DeleteMetaParameterCmd() {
  cout << "  > Enter variable name: ";
  string name;
  if (!GetNonCancelInputCmd(&name)) { return; }

  VarHandle handle(name, VarHandleSpec::META);
  PrintStatus(editor_->params().DeleteMetaVariable(handle));
}

void PlatoInterpreter::AddConstraintCmd() {
  cout << "  > Enter parameter handles (as OpName[SPACE]ParamName,OpName[Space]ParamName: ";

  vector<string> handle_strs;
  {
    string input;
    if (!GetNonCancelInputCmd(&input)) { return; }
    boost::split(handle_strs, input, boost::is_any_of(","));
  }

  vector<VarHandle> handles;
  for (const string& str : handle_strs) {
    vector<string> parts;
    boost::split(parts, str, boost::is_any_of(" "));
    if (parts.size() != 2) {
      cout << "  --> Error: invalid handle string '" << str << "'" << endl;
      return;
    }

    VarHandle handle(parts[0], parts[1], VarHandleSpec::PARAM);
    const VarInfo* info = editor_->params().GetVarInfo(handle);
    if (!info) {
      cout << "  --> Error: no parameter matches parsed handle: " << handle;
      return;
    }
    handles.push_back(handle);
  }

  cout << "  --> Available params for setting constraint: " << endl;
  for (int i = 0; i < handles.size(); ++i) {
    cout << "$" << i << "  -  " << handles[i] << endl;
  }

  cout << "  > Enter constraint expression: ";
  string input;
  if (!GetNonCancelInputCmd(&input)) { return; }

  PrintStatus(editor_->AddConstraint(input, handles));
}

bool PlatoInterpreter::PrintVariableValue(const VarInfo& info) const {
  cout << "----------------------------------------------------" << endl;
  cout << "Info : " << info << endl;
  cout << "Value: ";
  switch (info.type()) {
    case VarInfoSpec::TYPE_UNK:
      cout << "  --> Error: cannot get value for UNK type" << endl;
      return false;
    case VarInfoSpec::TYPE_INT32:
      cout << editor_->tpl()->params_container().
          Get<int32>(info.handle())<< endl;
      break;
    case VarInfoSpec::TYPE_DOUBLE:
      cout << editor_->tpl()->params_container().
          Get<double>(info.handle()) << endl;
      break;
    case VarInfoSpec::TYPE_BOOL:
      cout << editor_->tpl()->params_container().
          Get<bool>(info.handle()) << endl;
      break;
    case VarInfoSpec::TYPE_STRING:
      cout << editor_->tpl()->params_container().
          Get<const string&>(info.handle()) << endl;
      break;
    default:
      cout << endl << editor_->tpl()->params_container().
          Get<const Message&>(info.handle()).DebugString() << endl;
      break;
  }
  cout << "----------------------------------------------------" << endl;
  return true;
}

void PlatoInterpreter::PrintOperationVariables(const OperationNode* op) const {
  RegisteredOpSpec spec;
  CHECK(op->operation()->ToSpec(&spec));
  cout << "----------------------------------------------------" << endl;
  cout << spec.DebugString() << endl;
  cout << "----------------------------------------------------" << endl;
}

void PlatoInterpreter::GetValueCmd() const {
  PrintTreesCmd();
  cout << "  > Enter operation name: ";

  string op_name;
  if (!GetNonCancelInputCmd(&op_name)) { return; }

  const OperationNode* op = editor_->tpl()->FindNodeByName(op_name);
  if (!op) {
    cout << "  --> Error: could not find \"" << op_name << "\"" << endl;
    return;
  }

  // Print just this operation's parameters
  for (const auto& var_info : op->operation()->params().vars()) {
    PrintDetailedParameters(var_info);
  }
  cout << "  > Enter variable name [empty to print full spec]: ";

  string var_name;
  if (!GetNonCancelInputCmd(&var_name)) { return; }

  if (var_name.empty()) {
    PrintOperationVariables(op);
    PrintStatus(true);
  } else {
    VarHandle handle(op_name, var_name);
    const VarInfo* info = editor_->tpl()->params_container().GetVarInfo(handle);
    if (!info) {
      cout << "  --> Error: could not get info for var " << handle << endl;
      return;
    }
    PrintStatus(PrintVariableValue(*info));
  }
}

void PlatoInterpreter::SetValueCmd() const {
  cout << "OOPS, not implemented yet!" << endl;
}


}  // namespace mit_plato
