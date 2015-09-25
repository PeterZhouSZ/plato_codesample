#ifndef _FAB_GEOMETRY_TEMPLATE_LIBRARY_H__
#define _FAB_GEOMETRY_TEMPLATE_LIBRARY_H__

#include <map>
#include <set>
#include <string>
using std::string;

#include <boost/smart_ptr/scoped_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <glog/logging.h>

#include "fab/geometry/template/constraints.h"
#include "fab/geometry/template/constraints.pb.h"
#include "fab/geometry/template/operations.h"
#include "fab/geometry/template/operations.pb.h"


namespace mit_plato {

//! Encapsulates pre-specified primitives, including operations
//! for creating, modifying and combining shapes.
//! To register a new spec at compile time:
//!
//! RegisteredOpSpec MySpecialOpSpec() { ... }
//! REGISTER_OP_SPEC("My special op", MySpecialOpSpec);
class FabTemplateLibrary {
 public:
  typedef OpNodeProto (*OpSpecFunctionPtr)();
  typedef RegisteredConstraintSpec (*ConSpecFunctionPtr)();

  // Automatically adds --data_path elements to library
  static bool InitLibrary();

  // SUBCOMPONENTS -------------------------------------------------------------

  static void AddSubcomponentsPath(const string& absolute_path);

  static const std::set<string>& FoundSubcomponents();


  // CONSTRAINTS ---------------------------------------------------------------
  static const std::set<string>& RegisteredConstraints();

  // TODO: Avoid name collisions
  static RegisteredConstraintSpec Constraint(const string& name);

  // SHAPES --------------------------------------------------------------------

  //!
  //! Pre-defined specs for subclasses of CreateShapeOperation;
  //! must be registered via REGISTER_OP_SPEC
  static const std::set<string>& RegisteredShapes();

  //!
  //! Pre-defined specs for subclasses of ModifyShapeOperation;
  //! must be registered via REGISTER_OP_SPEC
  static const std::set<string>& RegisteredTransformations();

  //!
  //! Pre-defined specs for subclasses of CombineShapesOperation;
  //! must be registered via REGISTER_OP_SPEC
  static const std::set<string>& RegisteredCombineOperations();

  //!
  //! Creates spec for a registered CreateShapeOperation by name
  // TODO: handle failure appropriately
  //  static RegisteredOpSpec Shape(const string& name);
  static OpNodeProto Shape(const string& name);

  //!
  //! Creates spec for a registered ModifyShapeOperation by name
  // TODO: handle failure appropriately
  static RegisteredOpSpec Transformation(const string& name);

  //!
  //! Creates spec for a registered CombineShapesOperation by name
  // TODO: handle failure appropriately
  static RegisteredOpSpec CombineOperation(const string& name);

  // REGISTRATION --------------------------------------------------------------

  //!
  //! Do not call directly. Instead, use provided macros such as
  //! REGISTER_OP_SPEC
  template <class SpecClass>
  static bool RegisterSpecByName(const string& name,
                                 SpecClass (*CreateSpecFuncPtr)());

 private:
  bool InitLibraryInternal();

  template <class SpecClass>
  static bool CreateSpecByName(const string& name,
                               SpecClass* proto_spec);

  template <class SpecClass>
  static bool InternalRegisterSpecByName(const string& name,
                                         SpecClass (*CreateSpecFuncPtr)());


  // Gets the internal structure holding this particular type
  // of function pointer
  template <class FuncPtr>
  static std::map<string, FuncPtr>& GetFuncPtrMap();

  static FabTemplateLibrary& Singleton();
  FabTemplateLibrary() {}

  std::map<string, OpSpecFunctionPtr> op_specs_;
  std::set<string> create_op_specs_;
  std::set<string> modify_op_specs_;
  std::set<string> combine_op_specs_;

  std::map<string, ConSpecFunctionPtr> con_specs_;
  std::set<string> con_spec_names_;

  std::map<string, string> subtrees_;
  std::set<string> subtree_names_;

  boost::mutex lock_;
};

// Specializations for getting access to the internal map of
// registered function pointers for creating specs.
template <>
inline std::map<string, FabTemplateLibrary::OpSpecFunctionPtr>&
FabTemplateLibrary::GetFuncPtrMap<FabTemplateLibrary::OpSpecFunctionPtr>() {
  return Singleton().op_specs_;
}

template <>
inline std::map<string, FabTemplateLibrary::ConSpecFunctionPtr>&
FabTemplateLibrary::GetFuncPtrMap<FabTemplateLibrary::ConSpecFunctionPtr>() {
  return Singleton().con_specs_;
}

template <class SpecClass>
inline bool FabTemplateLibrary::CreateSpecByName(const string& name,
                                                 SpecClass* proto_spec) {
  boost::mutex::scoped_lock lock(Singleton().lock_);

  typedef SpecClass (*CreateSpecFuncPtr) ();
  typedef std::map<string, CreateSpecFuncPtr> FuncPtrMap;
  const FuncPtrMap& registered = GetFuncPtrMap<CreateSpecFuncPtr>();

  typename FuncPtrMap::const_iterator it = registered.find(name);
  if (it == registered.end()) {
    LOG(ERROR) << "Failed to create spec by name \"" << name << "\"";
    return false;
  }

  proto_spec->CopyFrom((*it->second)());
  return true;
}

template <class SpecClass>
inline bool FabTemplateLibrary::InternalRegisterSpecByName(
    const string& name,
    SpecClass (*create_spec_func_ptr)()) {
  VLOG(1) << "Registering spec " << name;

  typedef SpecClass (*CreateSpecFuncPtr) ();
  typedef std::map<string, CreateSpecFuncPtr> FuncPtrMap;
  FuncPtrMap& registered = GetFuncPtrMap<CreateSpecFuncPtr>();

  typename FuncPtrMap::const_iterator it = registered.find(name);
  if (it != registered.end()) {
    LOG(ERROR) << "Spec by name \"" << name << "\" already exists";
    return false;
  }
  registered[name] = create_spec_func_ptr;

  return true;
}

template <class SpecClass>
inline bool FabTemplateLibrary::RegisterSpecByName(
    const string& name,
    SpecClass (*create_spec_func_ptr)()) {
  boost::mutex::scoped_lock lock(Singleton().lock_);

  return InternalRegisterSpecByName<SpecClass>(name, create_spec_func_ptr);
}

template <>
inline bool FabTemplateLibrary::RegisterSpecByName<RegisteredConstraintSpec>(
    const string& name,
    RegisteredConstraintSpec (*create_spec_func_ptr)()) {
  boost::mutex::scoped_lock lock(Singleton().lock_);

  if (!InternalRegisterSpecByName(name, create_spec_func_ptr)) {
    return false;
  }

  Singleton().con_spec_names_.insert(name);
  return true;
}

// TODO: what to do if CREATE_SPEC_FUNC is not an identifier??
#define REGISTER_OP_SPEC(NAME, CREATE_SPEC_FUNC) \
namespace { \
bool __registered_##CREATE_SPEC_FUNC = \
  FabTemplateLibrary::RegisterSpecByName<OpNodeProto>( \
      NAME, &CREATE_SPEC_FUNC); \
}

#define REGISTER_CONSTRAINT_SPEC(NAME, CREATE_SPEC_FUNC) \
namespace { \
bool __registered_##CREATE_SPEC_FUNC = \
  FabTemplateLibrary::RegisterSpecByName<RegisteredConstraintSpec>( \
      NAME, &CREATE_SPEC_FUNC); \
}

}  // namespace mit_plato

#endif  // _FAB_GEOMETRY_TEMPLATE_LIBRARY_H__
