#ifndef _FAB_GEOMETRY_TEMPLATE_TEMPLATE_H__
#define _FAB_GEOMETRY_TEMPLATE_TEMPLATE_H__

#include <map>
#include <set>
#include <vector>
using std::map;
using std::set;
using std::vector;

#include <boost/smart_ptr/shared_ptr.hpp>

#include "fab/geometry/template/template.pb.h"
#include "fab/geometry/template/template_params.h"

namespace mit_plato {

class Constraint;
class FabGeometryTemplate;
class Operation;
class PropEvaluator;
class Shape;

// NAME TRACKER-----------------------------------------------------------------

// This is never reset, but this is ok
class OperationNameTracker {
 public:
  OperationNameTracker() : last_id_(0) {}

  // Resets global registered names and ids.
  static void GlobalReset();

  // Generates globally unique id.
  static int GetNextId();

  // Resets this instance's names and ids.
  void Reset();

  // Check-fails if the name already exists
  void RegisterName(const string& name);

  // Is guaranteed to create a name that has not been registered yet;
  // registers the returned name.
  string GenerateNextName(const Operation* operation);

  // Generates name, but does not register it.
  string GenerateNextNameDoNotRegister(const string& s);

 private:
  static OperationNameTracker& Singleton();

  std::map<string, int> created_ops_;
  std::set<string> used_names_;
  int last_id_;
};

// OPERATION NODE FACTORY ------------------------------------------------------

class OperationNode;

class OperationNodeFactory {
 public:
  OperationNodeFactory() {}

  static OperationNodeFactory& Singleton();

  OperationNode* Create(Operation* op, const string& name = "");

  //!
  //! Fails if the name has already been used.
  OperationNode* Create(const string& name,
                        bool allow_non_unique_name = false);

  void RegisterCreatedNode(const OperationNode* node);

  OperationNameTracker& tracker() { return tracker_; }

 private:
  OperationNameTracker tracker_;
};


// OPERATION NODE---------------------------------------------------------------

//! Utility class containing Operation with a unique name and
//! its children, as well as handles to all of the Operation's params.
class OperationNode {
 public:
  struct PtrIdCompare {
    bool operator() (const OperationNode* lhs,
                     const OperationNode* rhs) const {
      return lhs->id() < rhs->id();
    }
  };

  struct PtrIdEqulsPredicate {
    explicit PtrIdEqulsPredicate(int id) : id_(id) {}
    bool operator() (const OperationNode* node) {
      return node->id() == id_;
    }
   private:
    int id_;
  };

  struct PtrNameEqulsPredicate {
    explicit PtrNameEqulsPredicate(const string& name) : name_(name) {}
    bool operator() (const OperationNode* node) {
      return node->name() == name_;
    }
   private:
    const string name_;
  };

  ~OperationNode();

  bool ToSpec(OpNodeProto* spec) const;

  //!
  //! Every OperationNode has a unique name
  const string& name() const { return name_; }

  //!
  //! Every OperationNode has a unique id
  const int id() const { return id_; }

  Operation* operation() const { return operation_; }

  const OperationNode* parent() const { return parent_op_; }

  const vector<OperationNode*>& children() const { return child_ops_; }

  //!
  //! Returns the topmost parent node on this branch
  const OperationNode* TopOfBranch() const;

  //!
  //! Returns the bottommost child of this sequence of nodes
  //! (precedes the first downstream fork).
  const OperationNode* BottomOfBranch() const;

  // Finds the first fork in the descendant subtree and returns multiple
  // subtrees or leaves input vector empty.
  void GetDescendantSubtrees(vector<const OperationNode*>* subtrees) const;

  //!
  //! Returns all the leaves of this node's tree, including the node itself,
  //! if it is a leaf.
  void GetLeaves(vector<const OperationNode*>* leaves) const;

  // ! Returns true if its own params were modified or if any of the child
  // ! ops are dirty as well.
  bool is_dirty(bool clean_if_cached = true) const;

  void make_clean();
  void make_dirty();
  void make_parent_dirty();

  void Reset();
  string GetKey() const;

  // in milliseconds, keyed by the node configuration
  const map<string, double> run_times() const { return keyed_times_; }
  void clear_run_time() { keyed_times_.clear(); }

  void MockEvaluate(TemplateParams* container = NULL);

  boost::shared_ptr<const Shape> Evaluate(
      TemplateParams* container = NULL);

  boost::shared_ptr<const Shape> CachedShape() const;

  //! Sets cached shape to a new value and takes ownership;
  //! does not call make_clean automatically.
  void SetCachedShape(Shape* s);

  const Shape* shape() const { return shape_.get(); }

  // Mutators
  void AddChild(OperationNode* child) {
    child_ops_.push_back(child);
  }

  bool DeleteChild(OperationNode* child) {
    vector<OperationNode*>::iterator it =
        std::find(child_ops_.begin(), child_ops_.end(), child);
    if (it == child_ops_.end()) return false;

    child_ops_.erase(it);
    return true;
  }

  static void NamesToSuggestedNames(OpNodeProto* op_spec);
 private:
  //!
  //! Takes ownership of op; if name is empty, generates a new unique name.
  explicit OperationNode(Operation* op, const string& name);

  //!
  //! Fails if the name has already been used.
  explicit OperationNode(const string& name);

  // Disallow
  OperationNode();

  // Converts only the node and (NOT its children) to spec
  bool ToShallowSpec(OpNodeProto* spec) const;

  void AddToTime(double t) { keyed_times_[GetKey()] += t; }

  const string name_;
  const int id_;
  Operation* operation_;
  bool dirty_node_;

  OperationNode* parent_op_;
  vector<OperationNode*> child_ops_;

  boost::shared_ptr<Shape> shape_;
  bool cached_;  // true if cached shape was inserted (cleared by Reset)

  // Stores the sum of running times keyed by the node config at the time
  map<string, double> keyed_times_;

  friend class FabGeometryTemplate;
  friend class TemplateEditor;
  friend class OperationNodeFactory;
};


// TEMPLATE---------------------------------------------------------------------

//! Class representing the
class FabGeometryTemplate {
  friend class TemplateEditor;

 public:
  FabGeometryTemplate(OperationNode* node = NULL);
  ~FabGeometryTemplate();

  static FabGeometryTemplate* CheckCreateFromFile(const string& file);

  void CheckInstantiate();

  FabGeometryTemplate* Copy() const;

  const string& name() const { return name_; }

  // I/O operations ------------------------------------------------------------
  static FabGeometryTemplate* FromSpec(const FabTemplateProto& spec);
  bool ToSpec(FabTemplateProto* spec) const;

  // Customization -------------------------------------------------------------

  //!
  //! Instantiates geometry for the first time
  bool Instantiate();

  //!
  //! Runs update of necessary opearions give the parameter values that changed
  bool Update();

  //!
  //! Only useful for setting some internal params that result from an update
  bool MockUpdate();

  //!
  //! Returns geometry for all objects created by this template; will return
  //! objects that were created during a call to Instantiate/Update.
  const std::vector<boost::shared_ptr<const Shape> >& shapes() const;

  //!
  //! Interface for getting information about parameters and properties of
  //! the template.
  const TemplateParams& params_container() const { return params_; }

  //!
  //! Set parameter by name to a given value (must know type when calling)
  template <class T>
  bool SetParam(const VarHandle& handle, const T& value) {
    bool success = params_.Set(handle, value);
    return success;
  }

  //!
  //! Get value of the current parameter by name (must know type when calling)
  template <class T>
  const T GetParam(const VarHandle& handle) const {
    return params_.Get<T>(handle);
  }

  //!
  //! Gets a parameter of google::protobuf::Message subclass; NULL if fails
  template <class P>
  const P* GetProtoParam(const VarHandle& handle) const {
    return dynamic_cast<const P*>(
        &GetParam<const google::protobuf::Message&>(handle));
  }

  // Constraints & Solver ------------------------------------------------------
  //!
  //! Returns a list of all the constraints
  const vector<Constraint*>& constraints() const;

  //!
  //! Returns true if all constraints are satisfied given current param values.
  bool AllConstraintsSatisfied() const;

  //!
  //! Runs the solver on the template; may throw esceptions.
  void RunSolver();

  // Properties ----------------------------------------------------------------

  //!
  //! List of the handles of all requested properties.
  vector<VarInfo> properties() const;

  //!
  //! Evaluates a given propeerty and sets is_valid to true/false after checking
  //! if the property falls within its allowable bounds (if no bound set,
  //! is_valid will be set to true). NULL value not allowed.
  template <class T>
  const T EvaluateProperty(const VarHandle& handle,
                           bool* is_valid) const;

  // Misc Information ----------------------------------------------------------

  //!
  //! Returns total number of Operation nodes
  int num_operations() const;

  //!
  //! Returns factory that should be used for all node instantiations.
  OperationNodeFactory& node_factory() { return node_factory_; }

  //!
  //! Returns the root nodes of all the independent operation trees
  const vector<OperationNode*>& roots() const;

  const OperationNode* FindNodeById(const int id) const;

  const OperationNode* FindNodeByName(const string& name) const;

  // TODO: make accessible only to friends
  OperationNode* FindNodeByName(const string& name);

  // TODO: make accessible only to friends
  const set<OperationNode*>& nodes() const { return  nodes_; }

  void clear_run_time();

  // Needed for caching
  void RefreshNodeKeys();

 private:
  // Recursively creates the whole tree from spec, adds each node to nodes
  OperationNode* AddNodeFromSpec(const OpNodeProto& spec);

  // Node tree editing
  void AddRootNode(OperationNode* node);
  void AddNode(OperationNode* node);
  void RecursivelyAddNodes(OperationNode* node);
  void DeleteNode(OperationNode* node);

  // Constraint editing
  void AddConstraint(Constraint* constraint) {
    // TODO: find_if to see if constraint is redundant
    constraints_.push_back(constraint);
  }

  void AddGlobalProperty(PropEvaluator* prop);
  PropEvaluator* GetPropertyEvaluator(const VarHandle& handle) const;

  OperationNodeFactory node_factory_;

  vector<OperationNode*> root_ops_;
  vector<boost::shared_ptr<const Shape> > shapes_;

  set<OperationNode*> nodes_;
  TemplateParams params_;

  vector<Constraint*> constraints_;
  map<VarHandle, PropEvaluator*> props_;

  string name_;
};

}  // namespace mit_plato

#endif  //_FAB_GEOMETRY_TEMPLATE_TEMPLATE_H__
