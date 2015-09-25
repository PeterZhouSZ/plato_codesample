#ifndef _FAB_GEOMETRY_TEMPLATE_OPERATIONS_H__
#define _FAB_GEOMETRY_TEMPLATE_OPERATIONS_H__

#include <set>
#include <string>
#include <utility>
#include <vector>
using std::set;
using std::string;
using std::vector;

#include <boost/thread/mutex.hpp>

#include "fab/geometry/template/operations.pb.h"
#include "fab/geometry/template/params.h"
#include "fab/geometry/template/shape.h"
#include "fab/geometry/tri_mesh.h"


namespace google {
namespace protobuf {
class Message;
}  // namespace protobuf
}  // namespace google

namespace mit_plato {

class ProtoMessageWrapper;

// A unit operation as a part of the design creation process
class Operation {
 public:
  enum OpType { CREATE, MODIFY, COMBINE, IF_ELSE_IF, FOR};

  Operation();
  virtual ~Operation() {}

  static Operation* FromSpec(const RegisteredOpSpec& spec);
  bool ToSpec(RegisteredOpSpec* spec) const;

  //  properties()
  virtual void Reset() {}

  virtual const NamedVarContainer& params() const { return params_wrap_; }
  virtual NamedVarContainer& params() { return params_wrap_; }

  const google::protobuf::Message* proto() const { return params_wrap_.proto(); }

  // Returs if its own parameters were modified
  virtual bool is_dirty() const;

  // Sets its own parameters to "clean" state
  void make_clean() { params().make_clean(); }

  // Works for all registered subclasses
  const string& class_name() const;

  virtual OpType type() const = 0;

 protected:
  void ExposeParams(google::protobuf::Message* proto_params);

  template <class P>
  void InitParams(P* internal_spec, const P& spec = P()) {
    internal_spec->CopyFrom(spec);
    ExposeParams(internal_spec);
  }

  ProtoMessageWrapper params_wrap_;
  string name_;
};

// E.g. import from lib, create parametrized shape
class CreateShapeOperation : public Operation {
 public:

  // TODO: figure out memory conventions, consts, etc.
  // Maybe best is a shallow copy of Shape
  Shape* Create();

  virtual OpType type() const { return CREATE; }

 protected:
  // If set, group id will be added as an attribute to every face
  void AddTriangleRow(const vector<TriMesh::VertexHandle>& prev_pts,
                      const vector<TriMesh::VertexHandle>& new_pts,
                      TriMesh* mesh,
                      int group_id = 0);

  virtual Shape* CreateUncached() = 0;
};

// E.g. displacement mapping, etc.
class ModifyShapeOperation : public Operation {
 public:
  virtual void Modify(Shape*) = 0;

  virtual OpType type() const { return MODIFY; }
};

// E.g. join two shapes, position relative to each other
class CombineShapesOperation : public Operation {
 public:

  virtual Shape* Combine(const vector<const Shape*>&) = 0;

  virtual OpType type() const { return COMBINE; }
};


// REGISTRATION ----------------------------------------------------------------
typedef Operation* (*CreateOpFunctionPtr)(const RegisteredOpSpec&);
typedef bool (*EncodeOpFunctionPtr) (const Operation&, RegisteredOpSpec*);

class OperationRegisterer {
 public:

  // Do not call directly. Instead use macros in registration.h.
  static bool Register(const string& op_name,
                       const string& proto_name,
                       const Operation::OpType type,
                       CreateOpFunctionPtr create_fptr,
                       EncodeOpFunctionPtr encode_fptr);

  // Creates registered operation if the appropriate extension
  // in the spec had been set; returns NULL if unsuccessful
  static Operation* Create(const RegisteredOpSpec& spec);

  // Encodes registered operation as an extension in spec;
  // returns false if the operation is not registered or malformed
  static bool Encode(const Operation& op, RegisteredOpSpec* spec);

  // Returns the class name of the operation registered with this
  // proto name; returns some set value if operation not registered
  static const string& OperationNameFromSpec(const string& proto_name);

  //!
  // Returns class names of all CreateOperation subclasses
  static const std::set<string>& CreateOperations();

  //!
  // Returns class names of all ModifyOperation subclasses
  static const std::set<string>& ModifyOperations();

  //!
  // Returns class names of all CombineOperation subclasses
  static const std::set<string>& CombineOperations();

 private:
  static OperationRegisterer& Singleton();
  OperationRegisterer() {}

  typedef string ProtoSpecClassName;
  typedef string OperationClassName;

  std::map<OperationClassName,
           std::pair<CreateOpFunctionPtr, EncodeOpFunctionPtr> > registered_;

  std::map<ProtoSpecClassName, OperationClassName> registered_specs_;

  std::set<string> create_ops_;
  std::set<string> modify_ops_;
  std::set<string> combine_ops_;
  boost::mutex lock_;
};

}  // namespace mit_plato

#endif  // _FAB_GEOMETRY_TEMPLATE_OPERATIONS_H__
