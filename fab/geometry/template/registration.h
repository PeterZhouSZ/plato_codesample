#ifndef _FAB_GEOMETRY_TEMPLATE_REGISTRATION_H__
#define _FAB_GEOMETRY_TEMPLATE_REGISTRATION_H__

#include "fab/geometry/template/constraints.h"
#include "fab/geometry/template/library.h"
#include "fab/geometry/template/operations.h"
#include "fab/geometry/template/template_params.h"


namespace mit_plato {

// OPERATIONS ------------------------------------------------------------------

//! Call this registration to register Operation and its proto spec
//! by name so that it can be created from Spec and written to spec.
#define REGISTER_OPERATION(OpSubclass, OpProto) \
Operation* __Create_Registered_##OpSubclass( \
      const mit_plato::RegisteredOpSpec& spec) { \
  if (spec.HasExtension(OpProto::spec)) { \
    return new OpSubclass(spec.GetExtension(OpProto::spec)); \
  } \
  return NULL; \
} \
bool __Encode_Registered_##OpSubclass( \
    const Operation& op, mit_plato::RegisteredOpSpec* spec) { \
  if (!op.proto()) return false; \
  spec->MutableExtension(OpProto::spec)->CopyFrom(*op.proto()); \
  return true; \
} \
namespace { \
OpSubclass __tmp_##OpSubclass; \
bool __registered_##OpSubclass = OperationRegisterer::Register( \
    #OpSubclass, #OpProto, __tmp_##OpSubclass.type(), \
    &__Create_Registered_##OpSubclass, \
    &__Encode_Registered_##OpSubclass); \
}


//! Call this registration to register as by REGISTER_OPERATION and
//! to also save the default Spec of this operation in the FabTemplateLibrary
//! under a given Name (should be a string).
#define REGISTER_OPERATION_WITH_DEFAULT(Name, OpSubclass, OpProto) \
REGISTER_OPERATION(OpSubclass, OpProto); \
mit_plato::OpNodeProto __Create_Default_Spec_##OpSubclass() {  \
    mit_plato::OpNodeProto node_proto; \
    mit_plato::RegisteredOpSpec* spec = node_proto.mutable_op();    \
    CHECK(__Encode_Registered_##OpSubclass(OpSubclass(), spec)) \
    << "Failed to encode " << #OpSubclass << "(); " \
    << "the most likely cause is default constructor not calling " \
    << "ExposeParams"; \
    return node_proto; \
} \
REGISTER_OP_SPEC(Name, __Create_Default_Spec_##OpSubclass)


// CONSTRAINTS -----------------------------------------------------------------
#define REGISTER_CONSTRAINT(Name, ClassName) \
mit_plato::Constraint* __Create_Constraint_##ClassName( \
    const vector<ExprNode*>& exprs) { \
  return new ClassName(exprs); \
} \
namespace { \
bool __registered_constraint_##Name = \
  mit_plato::ConstraintRegister::Register( \
      #Name, __Create_Constraint_##ClassName); \
} \
RegisteredConstraintSpec __Create_Constraint_Spec_##Name() { \
  RegisteredConstraintSpec spec; \
  spec.set_type(#Name); \
  return spec; \
} \
REGISTER_CONSTRAINT_SPEC(#Name, __Create_Constraint_Spec_##Name)

}  // namespace mit_plato

#endif  // _FAB_GEOMETRY_TEMPLATE_REGISTRATION_H__
