#include "fab/geometry/template/operations.h"

#include <utility>
#include <boost/smart_ptr/scoped_ptr.hpp>
#include <glog/logging.h>
#include <google/protobuf/message.h>

#include "fab/geometry/template/expression.h"
#include "fab/geometry/template/registration.h"

namespace mit_plato {

// OPERATION -------------------------------------------------------------------
Operation::Operation() : params_wrap_(VarHandleSpec::PARAM) {}

Operation* Operation::FromSpec(const RegisteredOpSpec& spec) {
  return OperationRegisterer::Create(spec);
}

const string& Operation::class_name() const {
  return OperationRegisterer::OperationNameFromSpec(params_wrap_.proto_type());
}

bool Operation::ToSpec(RegisteredOpSpec* spec) const {
  return OperationRegisterer::Encode(*this, spec);
}

void Operation::ExposeParams(google::protobuf::Message* proto_params) {
  params_wrap_.Reset(proto_params);
}

bool Operation::is_dirty() const {
  return params().is_dirty();
}

Shape* CreateShapeOperation::Create() {
  return CreateUncached();
}

void CreateShapeOperation::AddTriangleRow(
    const vector<TriMesh::VertexHandle>& prev_pts,
    const vector<TriMesh::VertexHandle>& new_pts,
    TriMesh* mesh,
    int group_id) {
  if (prev_pts.empty() || new_pts.empty()) return;

  int num_pts = static_cast<int>(prev_pts.size());
  for (int pt_idx = 0; pt_idx <  num_pts; ++pt_idx) {
    vector<TriMesh::VertexHandle> face_vhandles;
    face_vhandles.push_back(prev_pts[pt_idx]);
    face_vhandles.push_back(prev_pts[(pt_idx + 1) % num_pts]);
    face_vhandles.push_back(new_pts[pt_idx]);
    mesh->smarterAddFace(face_vhandles, group_id);

    face_vhandles.clear();
    face_vhandles.push_back(prev_pts[(pt_idx + 1) % num_pts]);
    face_vhandles.push_back(new_pts[(pt_idx + 1) % num_pts]);
    face_vhandles.push_back(new_pts[pt_idx]);
    mesh->smarterAddFace(face_vhandles, group_id);
  }
}

// REGISTERER ------------------------------------------------------------------
OperationRegisterer& OperationRegisterer::Singleton() {
  static OperationRegisterer registerer;
  return registerer;
}

const set<string>& OperationRegisterer::CreateOperations() {
  return Singleton().create_ops_;
}

const set<string>& OperationRegisterer::ModifyOperations() {
  return Singleton().modify_ops_;
}

const set<string>& OperationRegisterer::CombineOperations() {
  return Singleton().combine_ops_;
}

Operation* OperationRegisterer::Create(const RegisteredOpSpec& spec) {
  // TODO: Check for number of set fields

  Operation* op = NULL;
  for (const auto& reg_pair : Singleton().registered_) {
    op = (*(reg_pair.second.first))(spec);
    if (op) break;
  }
  return op;
}

bool OperationRegisterer::Encode(
    const Operation& op, RegisteredOpSpec* spec) {
  auto it = Singleton().registered_.find(op.class_name());
  if (it == Singleton().registered_.end()) {
    LOG(ERROR) << "Operation " << op.class_name() << " not registered.";
    return false;
  }

  return (*(it->second.second))(op, spec);
}

bool OperationRegisterer::Register(const string& op_name,
                                   const string& proto_name,
                                   const Operation::OpType type,
                                   CreateOpFunctionPtr create_fptr,
                                   EncodeOpFunctionPtr encode_fptr) {
  boost::mutex::scoped_lock lock(Singleton().lock_);

  VLOG(1) << "Registering " << op_name << "(" << type << ")  with proto spec "
          << proto_name;
  std::map<string, string>::const_iterator it =
      Singleton().registered_specs_.find(proto_name);
  if (it != Singleton().registered_specs_.end()) {
    CHECK_EQ(it->second, op_name)
        << "Cannot register more than one Operation subclass "
        << "with the same protocol buffer definition (else "
        << "ambiguity will result): "
        << proto_name << " already registered.";
    return false;
  }
  Singleton().registered_[op_name] = std::make_pair(create_fptr, encode_fptr);
  Singleton().registered_specs_[proto_name] = op_name;

  // Deduce the type of the operation
  if (type == Operation::CREATE) {
    Singleton().create_ops_.insert(op_name);
  } else if (type == Operation::MODIFY) {
    Singleton().modify_ops_.insert(op_name);
  } else if (type == Operation::COMBINE) {
    Singleton().combine_ops_.insert(op_name);
  } else {
    VLOG(1) << "Unknown operation type: " << type
            << " for op: " << op_name;
  }

  return true;
}

const string& OperationRegisterer::OperationNameFromSpec(
    const string& proto_name) {
  boost::mutex::scoped_lock lock(Singleton().lock_);

  static const string not_found = "NOT_REGISTERED";
  std::map<string, string>::const_iterator it =
      Singleton().registered_specs_.find(proto_name);
  if (it != Singleton().registered_specs_.end()) {
    return it->second;
  }
  return not_found;
}

}  // namespace mit_plato
