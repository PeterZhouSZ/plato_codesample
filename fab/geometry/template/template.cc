#include <algorithm>
#include <iostream>
#include <sstream>
using std::stringstream;

#include <boost/timer/timer.hpp>
#include <boost/regex.hpp>
#include <gecode/search.hh>
#include <google/protobuf/text_format.h>

#include "fab/geometry/template/checks.h"
#include "fab/geometry/template/constraints.h"
#include "fab/geometry/template/create_operations.h"
#include "fab/geometry/template/search_paths.h"
#include "fab/geometry/template/model.h"
#include "fab/geometry/template/modify_operations.h"
#include "fab/geometry/template/operations.h"
#include "fab/geometry/template/special_operations.h"
#include "fab/geometry/template/template.h"
#include "util/proto_util.h"

DEFINE_bool(enable_analytics, false,
            "If true, keeps analytics at all the nodes.");

namespace mit_plato {

////////////////////////////////////////////////////////////////////////////////
// SOME CODE REMOVED
////////////////////////////////////////////////////////////////////////////////

// NAME TRACKER-----------------------------------------------------------------

OperationNameTracker& OperationNameTracker::Singleton() {
  static OperationNameTracker tracker;
  return tracker;
}

void OperationNameTracker::Reset() {
  created_ops_.clear();
  used_names_.clear();
  last_id_ = 0;
}

void OperationNameTracker::GlobalReset() {
  Singleton().Reset();
}

void OperationNameTracker::RegisterName(const string& name) {
  LOG(INFO) << "Registering name: " << name;
  CHECK(used_names_.find(name) == used_names_.end())
      << "Name \"" << name << "\" already used.";
  used_names_.insert(name);
}

string OperationNameTracker::GenerateNextNameDoNotRegister(
    const string& s) {
  boost::regex re("[^a-zA-Z0-9_\\-]");
  const string prefix = boost::regex_replace(s, re, "");

  int i = 0;
  std::stringstream name;
  do {
    name.str("");
    name << prefix << ++i;
  } while (used_names_.find(name.str()) != used_names_.end());
  return name.str();
}

string OperationNameTracker::GenerateNextName(const Operation* operation) {
  static const string null_name = "NULL_OP";
  const string& op_class = operation ? operation->class_name() : null_name;

  boost::regex re("^(Create)?(.*)");
  boost::regex re2("Operation$");
  const string prefix = boost::regex_replace(
      boost::regex_replace(op_class, re, "$2"), re2, "");

  std::stringstream name;
  do {
    name.str("");
    name << prefix << ++created_ops_[op_class];
  } while (used_names_.find(name.str()) != used_names_.end());
  used_names_.insert(name.str());
  return name.str();
}

int OperationNameTracker::GetNextId() {
  return ++Singleton().last_id_;
}

// OPERATION NODE FACTORY ------------------------------------------------------

OperationNodeFactory& OperationNodeFactory::Singleton() {
  static OperationNodeFactory factory;
  return factory;
}

OperationNode* OperationNodeFactory::Create(
    Operation* op, const string& name) {
  if (!name.empty()) {
    tracker_.RegisterName(name);
  }

  return new OperationNode(
      op, (!name.empty() ? name : tracker_.GenerateNextName(op)));
}

OperationNode* OperationNodeFactory::Create(const string& name,
                                            bool allow_non_unique_name) {
  if (!allow_non_unique_name)
    tracker_.RegisterName(name);

  return new OperationNode(name);
}

void OperationNodeFactory::RegisterCreatedNode(const OperationNode* node) {
  tracker_.RegisterName(node->name());
}

// OPERATION NODE---------------------------------------------------------------
OperationNode::OperationNode()
    : operation_(NULL),
      parent_op_(NULL),
      id_(0),
      dirty_node_(true),
      cached_(false) {}

OperationNode::OperationNode(const string& name)
    : operation_(NULL),
      parent_op_(NULL),
      id_(0),
      name_(name),
      dirty_node_(true),
      cached_(false) {}

OperationNode::OperationNode(Operation* op, const string& name)
    : operation_(op),
      parent_op_(NULL),
      id_(OperationNameTracker::GetNextId()),
      name_(name),
      dirty_node_(true),
      cached_(false) {
  if (operation_) {
    operation_->params().set_name_space(name_);
  }
}

OperationNode::~OperationNode() {
  delete operation_;
}

void OperationNode::Reset() {
  if (operation_) { operation_->Reset(); }
  cached_ = false;
}

bool OperationNode::is_dirty(bool clean_if_cached) const {
  if (!operation_) return false;

  if (dirty_node_) return true;

  if (clean_if_cached && cached_) return false;

  VLOG(3) << "Operation " << name() << " operation "
          << (operation_->is_dirty() ? "DIRTY" : "CLEAN");

  if (operation_->is_dirty()) return true;

  for (OperationNode* child : child_ops_) {
    if (child->is_dirty()) {
      return true;
    }
  }
  return false;
}

void OperationNode::make_clean() {
  operation_->make_clean();
  dirty_node_ = false;
}

void OperationNode::make_dirty() {
  dirty_node_ = true;
}

void OperationNode::make_parent_dirty() {
  if (parent_op_) {
    parent_op_->make_dirty();
  }
}

void OperationNode::GetLeaves(vector<const OperationNode*>* leaves) const {
  if (child_ops_.empty()) {
    leaves->push_back(this);
  }

  for (const OperationNode* child : child_ops_) {
    child->GetLeaves(leaves);
  }
}

void OperationNode::GetDescendantSubtrees(
    vector<const OperationNode*>* subtrees) const {
  if (child_ops_.empty()) return;

  if (child_ops_.size() == 1) {
    child_ops_[0]->GetDescendantSubtrees(subtrees);
  } else {
    subtrees->insert(subtrees->begin(), child_ops_.begin(), child_ops_.end());
  }
}

const OperationNode* OperationNode::TopOfBranch() const {
  if (parent_op_ && parent_op_->operation()->type() == Operation::MODIFY) {
    return parent_op_->TopOfBranch();
  }
  return this;
}

const OperationNode* OperationNode::BottomOfBranch() const {
  if (child_ops_.size() == 1) {
    return child_ops_[0]->BottomOfBranch();
  }
  return this;
}

void OperationNode::SetCachedShape(Shape* s) { shape_.reset(s); cached_ = true;}

boost::shared_ptr<const Shape> OperationNode::CachedShape() const {
  return shape_;
}

namespace {
double GetTimerMs(boost::timer::cpu_timer& timer) {
  auto nanoseconds = boost::chrono::nanoseconds(timer.elapsed().user + timer.elapsed().system);
  auto milliseconds = boost::chrono::duration_cast<boost::chrono::milliseconds>(nanoseconds);
  return milliseconds.count();
}
}

void OperationNode::MockEvaluate(TemplateParams* container) {
  if (operation_->type() == Operation::FOR) {
    ForOperation* for_op = dynamic_cast<ForOperation*>(operation_);
    CHECK(for_op);
    CHECK(for_op->Init(container));

    while (for_op->Loop()) {
      child_ops_[0]->MockEvaluate(container);
    }
    for_op->CleanUp();
  } else {
    for (int i = 0; i < child_ops_.size(); ++i) {
      child_ops_[i]->MockEvaluate(container);
    }
  }
}

boost::shared_ptr<const Shape> OperationNode::Evaluate(
    TemplateParams* container) {
  // TODO: Fix this!
  // No caching is possible for If/ELSE node, b/c the value
  // of case expressions could have changed & they need to be
  // reevaluated.

  if (!is_dirty() && shape_) {
    LOG(INFO) << name() << ": returning CACHED result, with: ";
    shape_->log_info();
    return shape_;
  }

  if (is_dirty()) {
    clear_run_time();
  }

  if (operation_->type() == Operation::IF_ELSE_IF) {
    IfElseIfOperation* if_op = dynamic_cast<IfElseIfOperation*>(operation_);
    CHECK(if_op);

    int matched_case = 0;
    if (!if_op->GetCase(*container, &matched_case)) {  // TODO: need to pass
      LOG(INFO) << name() << ": no cases matched.";
      shape_.reset(new Shape());
    } else {
      CHECK_LE(matched_case, child_ops_.size())
          << "Malformed if node: " << name();
      shape_.reset(
          new Shape(*child_ops_[matched_case]->Evaluate(
              CHECK_NOTNULL(container))));
    }
  } else if (operation_->type() == Operation::FOR) {
    CHECK_EQ(1, child_ops_.size()) << "Malformed for node: " << name();
    ForOperation* for_op = dynamic_cast<ForOperation*>(operation_);
    CHECK(for_op);
    CHECK(for_op->Init(container));

    shape_.reset(new Shape());
    while (for_op->Loop()) {
      boost::shared_ptr<const Shape> child_res =
          child_ops_[0]->Evaluate(container);

      if (FLAGS_enable_analytics) {
        boost::timer::cpu_timer timer;
        shape_->MutableMesh().AddData(child_res->Mesh());
        AddToTime(GetTimerMs(timer));
      } else {
        shape_->MutableMesh().AddData(child_res->Mesh());
      }
    }
    for_op->CleanUp();
  } else if (operation_->type() == Operation::CREATE) {
    CHECK(child_ops_.empty()) << "Malformed create node: " << name();
    CreateShapeOperation* create_op =
        dynamic_cast<CreateShapeOperation*>(operation_);
    CHECK(create_op);

    if (FLAGS_enable_analytics) {
      boost::timer::cpu_timer timer;
      shape_.reset(create_op->Create());
      AddToTime(GetTimerMs(timer));
    } else {
      shape_.reset(create_op->Create());
    }
  } else if (operation_->type() == Operation::MODIFY) {
    CHECK_EQ(1, child_ops_.size()) << "Malformed modify node: " << name();
    ModifyShapeOperation* modify_op =
        dynamic_cast<ModifyShapeOperation*>(operation_);
    CHECK(modify_op);
    shape_.reset(new Shape(
        *CHECK_NOTNULL(child_ops_[0]->Evaluate(container).get())));
    if (FLAGS_enable_analytics) {
      boost::timer::cpu_timer timer;
      modify_op->Modify(shape_.get());
      AddToTime(GetTimerMs(timer));
    } else {
      modify_op->Modify(shape_.get());
    }
  } else {
    CHECK_GE(child_ops_.size(), 1) << "Malformed combine node: " << name();
    CombineShapesOperation* combine_op =
        dynamic_cast<CombineShapesOperation*>(operation_);
    CHECK(combine_op);

    vector<const Shape*> shapes;
    for (int i = 0; i < child_ops_.size(); ++i) {
      shapes.push_back(
          CHECK_NOTNULL(child_ops_[i]->Evaluate(container).get()));
    }
    LOG(INFO) << name() << ": evaluated " << shapes.size() << " children";

    if (FLAGS_enable_analytics) {
      boost::timer::cpu_timer timer;
      shape_.reset(combine_op->Combine(shapes));
      AddToTime(GetTimerMs(timer));
    } else {
      shape_.reset(combine_op->Combine(shapes));
    }
  }

  make_clean();

  LOG(INFO) << name() << ": returning NEW result";
  shape_->log_info();
  return shape_;
}

bool OperationNode::ToShallowSpec(OpNodeProto* spec) const {
  spec->Clear();
  if (!operation_) return true;

  if (!operation_->ToSpec(spec->mutable_op())) {
    LOG(ERROR) << "Could not create spec for node " << name();
    return false;
  }
  spec->set_name(name_);
  return true;
}

bool OperationNode::ToSpec(OpNodeProto* spec) const {
  VLOG(4) << "Node " << name() << " to spec";
  if (!ToShallowSpec(spec)) {
    return false;
  }

  for (const OperationNode* child_node : children()) {
    if (!child_node->ToSpec(spec->add_child())) {
      return false;
    }
  }
  return true;
}

// TEMPLATE---------------------------------------------------------------------

FabGeometryTemplate::FabGeometryTemplate(OperationNode* node) {
  if (!node) return;

  root_ops_.push_back(node);
  RecursivelyAddNodes(node);

  for (const OperationNode* node : nodes_) {
    node_factory_.RegisterCreatedNode(node);
  }
}

FabGeometryTemplate* FabGeometryTemplate::Copy() const {
  FabTemplateProto spec;
  CHECK(ToSpec(&spec));
  return FabGeometryTemplate::FromSpec(spec);
}

FabGeometryTemplate::~FabGeometryTemplate() {
  for (OperationNode* node : nodes_) {
    delete node;
  }

  for (Constraint* con : constraints_) {
    delete con;
  }

  for (const auto& prop : props_) {
    delete prop.second;
  }
  LOG(INFO) << "Deleted template";
}

FabGeometryTemplate* FabGeometryTemplate::CheckCreateFromFile(const string& file) {
  string absolute_path;
  CHECK(SearchPaths::FindFile(
      file, &absolute_path)) << "Could not find input file: " << file;

  FabTemplateProto tpl_proto;
  CHECK(mds_util::ReadProtoFromFile(absolute_path, &tpl_proto));
  return CHECK_NOTNULL(FabGeometryTemplate::FromSpec(tpl_proto));
}

void FabGeometryTemplate::CheckInstantiate() {
  try {
    CHECK(Instantiate());
  } catch (runtime_error& e) {
    LOG(FATAL) << "Could not instantiate: " << e.what();
  }
  CHECK_GT(shapes().size(), 0);
}

int FabGeometryTemplate::num_operations() const {
  return nodes_.size();
}

const vector<OperationNode*>& FabGeometryTemplate::roots() const {
  return root_ops_;
}

const vector<Constraint*>& FabGeometryTemplate::constraints() const {
  return constraints_;
}

const std::vector<boost::shared_ptr<const Shape> >&
FabGeometryTemplate::shapes() const {
  return shapes_;
}

vector<VarInfo> FabGeometryTemplate::properties() const {
  vector<VarInfo> result;
  for (const auto& p : props_) {
    result.push_back(VarInfo(p.first, p.second->type()));
  }
  return result;
}

PropEvaluator* FabGeometryTemplate::GetPropertyEvaluator(
    const VarHandle& handle) const {
  map<VarHandle, PropEvaluator*>::const_iterator it = props_.find(handle);
  if (it == props_.end()) {
    throw runtime_error("Failed to get property by handle " + handle.full_name());
  }
  return it->second;
}

void FabGeometryTemplate::AddGlobalProperty(PropEvaluator* prop) {
  VarHandle handle(prop->id(), VarHandleSpec::PROP);
  if (props_.find(handle) != props_.end()) {
    LOG(ERROR) << "Attempting to add global property with handle "
               << handle << " twice";
    return;
  }
  props_[handle] = prop;
}

template <>
const int32 FabGeometryTemplate::EvaluateProperty<int32>(
    const VarHandle& handle, bool* is_valid) const {
  PropEvaluator* prop = GetPropertyEvaluator(handle);
  return prop->EvaluateInt(root_ops_, is_valid);
}

template <>
const double FabGeometryTemplate::EvaluateProperty<double>(
    const VarHandle& handle, bool* is_valid) const {
  PropEvaluator* prop = GetPropertyEvaluator(handle);
  return prop->EvaluateDouble(root_ops_, is_valid);
}

template <>
const bool FabGeometryTemplate::EvaluateProperty<bool>(
    const VarHandle& handle, bool* is_valid) const {
  if (handle == VarHandle("constraints_ok", VarHandleSpec::PROP)) {
    *is_valid  = AllConstraintsSatisfied();
    return *is_valid;
  }
  PropEvaluator* prop = GetPropertyEvaluator(handle);
  return prop->EvaluateBool(root_ops_, is_valid);
}

bool FabGeometryTemplate::AllConstraintsSatisfied() const {
  for (const Constraint* con : constraints()) {
    if (con->IsEnabled() && !con->IsSatisfied()) return false;
  }
  return true;
}

void FabGeometryTemplate::RunSolver() {
  LOG(INFO) << "Creating solver...";
  boost::scoped_ptr<TemplateModel> model(new TemplateModel(this));
  Gecode::DFS<TemplateModel> solver(model.get());

  LOG(INFO) << "Running solver...";
  TemplateModel* solution = NULL;
  try {
    solution = solver.next();
  } catch (...) {
    LOG(ERROR) << "Failed when running solver.next()!";
    throw runtime_error("No solution!");
  }
  if (solution) {
    LOG(INFO) << "Got solution!";
    try {
      solution->SetSolution(this);
      delete solution;
      LOG(INFO) << "Set solution!";
    } catch (std::exception& ex) {
      delete solution;
      stringstream ss;
      ss << "Underdetermined system: " << ex.what();
      throw runtime_error(ss.str());
    }
  } else {
    throw runtime_error("No solution!");
  }
}

void FabGeometryTemplate::RefreshNodeKeys() {
  MockUpdate();
  for (OperationNode* node : nodes_) {
    if (node->is_dirty(false)) {
      node->Reset();
    }
  }
}

bool FabGeometryTemplate::Instantiate() {
  VLOG(1) << "Instantiate called";
  shapes_.clear();
  if (root_ops_.empty()) { return false; }   // TODO: remove

  bool success = true;
  for (OperationNode* root_op : root_ops_) {
    boost::shared_ptr<const Shape> shape = root_op->Evaluate(&params_);
    if (!shape) {
      success = false;
      LOG(ERROR) << "Failed to evaluate root operation " << root_op->name();
    } else {
      shapes_.push_back(shape);
    }
  }
  return success;
}

bool FabGeometryTemplate::MockUpdate() {
  for (OperationNode* root_op : root_ops_) {
    root_op->MockEvaluate(&params_);
  }
}

bool FabGeometryTemplate::Update() {
  // TODO: smarter implementation
  return Instantiate();
}

void FabGeometryTemplate::AddRootNode(OperationNode* node) {
  root_ops_.push_back(node);
  AddNode(node);
}

void FabGeometryTemplate::AddNode(OperationNode* node) {
  if (!node) return;

  if (nodes_.find(node) != nodes_.end()) {
    LOG(INFO) << "Skipping adding Op; already present;";
    return;
  }

  nodes_.insert(node);
  params_.AddOperationParams(node->name(), node->operation());
}

void FabGeometryTemplate::DeleteNode(OperationNode* node) {
  if (!node) return;

  vector<OperationNode*>::iterator found =
      find(root_ops_.begin(), root_ops_.end(), node);
  if (found != root_ops_.end()) {
    root_ops_.erase(found);
  }
  nodes_.erase(node);
  params_.RemoveOperationParams(node->name());
  delete node;
}

void FabGeometryTemplate::RecursivelyAddNodes(OperationNode* node) {
  if (!node) return;
  AddNode(node);
  for (OperationNode* child : node->child_ops_) {
    RecursivelyAddNodes(child);
  }
}

OperationNode* FabGeometryTemplate::AddNodeFromSpec(const OpNodeProto& spec) {
  Operation* op = Operation::FromSpec(spec.op());
  CHECK(op) << "Failed to create operation from spec: " << spec.DebugString();

  string name;
  if (spec.has_name()) {
    name = spec.name();
  } else if (spec.has_suggested_name()) {
    name = node_factory_.tracker().GenerateNextNameDoNotRegister(
        spec.suggested_name());
  }
  OperationNode* res = node_factory_.Create(op, name);
  for (const OpNodeProto& child_spec : spec.child()) {
    OperationNode* child = AddNodeFromSpec(child_spec);
    child->parent_op_ = res;
    res->child_ops_.push_back(child);
  }
  AddNode(res);
  return res;
}

FabGeometryTemplate* FabGeometryTemplate::FromSpec(
    const FabTemplateProto& spec) {
  FabGeometryTemplate* tpl = new FabGeometryTemplate();
  tpl->name_ = spec.name();
  for (const OpNodeProto& root_spec : spec.root_op()) {
    tpl->root_ops_.push_back(tpl->AddNodeFromSpec(root_spec));
  }

  if (!tpl->params_.InitFromSpec(spec)) {
    LOG(ERROR) << "Failed to initialized params from spec.";
    delete tpl;
    return NULL;
  }

  for (const RegisteredConstraintSpec& con_spec : spec.constraint()) {
    string error;
    Constraint* con = ConstraintParser<TemplateParams>::ParseFromSpec(
        con_spec, &tpl->params_, &error);
    if (!con) {
      LOG(ERROR) << error;
      LOG(ERROR) << con_spec.DebugString();
      delete tpl;
      return NULL;
    }
    tpl->AddConstraint(con);
  }

  for (const PropEvaluatorSpec& prop_spec : spec.global_prop()) {
    PropEvaluator* prop = PropEvaluator::FromSpec(prop_spec);
    if (!prop) {
      LOG(ERROR) << "Failed to create prop from spec: "
                 << prop_spec.DebugString();
      delete tpl;
      return NULL;
    }
    tpl->AddGlobalProperty(prop);
  }

  return tpl;
}

bool FabGeometryTemplate::ToSpec(FabTemplateProto* spec) const {
  CHECK_NOTNULL(spec)->Clear();
  bool success = true;

  // Write operation tree
  for (const OperationNode* root_op : root_ops_) {
    success = success && root_op->ToSpec(spec->add_root_op());
  }

  // Write variable annotations
  success = success && params_.ToSpec(spec);

  for (Constraint* con : constraints_) {
    success = success && con->ToSpec(spec->add_constraint());
  }

  for (const auto& p : props_) {
    success = success && p.second->ToSpec(spec->add_global_prop());
  }

  return success;
}

void FabGeometryTemplate::clear_run_time() {
  for (OperationNode* node : nodes_) {
    node->clear_run_time();
  }
}

const OperationNode* FabGeometryTemplate::FindNodeById(const int id) const {
  set<OperationNode*>::const_iterator it = std::find_if(
      nodes_.begin(), nodes_.end(), OperationNode::PtrIdEqulsPredicate(id));
  if (it == nodes_.end()) {
    LOG(ERROR) << "Could not find node with ID: " << id;
    return NULL;
  }
  return *it;
}

const OperationNode* FabGeometryTemplate::FindNodeByName(
    const string& name) const {
  set<OperationNode*>::const_iterator it = std::find_if(
      nodes_.begin(), nodes_.end(), OperationNode::PtrNameEqulsPredicate(name));
  if (it == nodes_.end()) {
    LOG(ERROR) << "Could not find node with name \"" << name << "\"";
    return NULL;
  }
  return *it;
}

OperationNode* FabGeometryTemplate::FindNodeByName(const string& name) {
  set<OperationNode*>::const_iterator it = std::find_if(
      nodes_.begin(), nodes_.end(), OperationNode::PtrNameEqulsPredicate(name));
  if (it == nodes_.end()) {
    LOG(ERROR) << "Could not find node with name \"" << name << "\"";
    return NULL;
  }
  return *it;
}

}  // namespace mit_plato
