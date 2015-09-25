#ifndef _FAB_GEOMETRY_TEMPLATE_PARAMS_H__
#define _FAB_GEOMETRY_TEMPLATE_PARAMS_H__

#include <iostream>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
using std::runtime_error;
using std::set;
using std::string;

#include <glog/logging.h>
#include <google/protobuf/stubs/common.h>
using google::protobuf::int32;

#include "fab/geometry/template/operations.pb.h"

namespace mit_plato {

// PARAM HANDLE ----------------------------------------------------------------

//! A handle to the parameter stored in the global repository.
//! (Must contain operation_name to be fully specified.)
struct VarHandle {
  static const char nested_delimiter;
  static const char field_num_delimiter;

  typedef VarHandleSpec Spec;
  typedef Spec::HandleKind HandleKind;

  VarHandle() : kind(Spec::UNK) {}

  VarHandle(const string& vname_space,
            const string& vname,
            HandleKind handlekind = Spec::UNK)
      : name_space(vname_space), var(vname),
        kind(handlekind) {}

  VarHandle(const VarHandle& handle)
      : kind(handle.kind), name_space(handle.name_space),
        var(handle.var) {}

  VarHandle(const VarHandleSpec& spec)
      : name_space(spec.name_space()), var(spec.name()),
        kind(spec.kind()) {}

  //!
  //! An intentionally implicit constructor from string
  VarHandle(const string& vname,
            HandleKind handlekind = Spec::UNK)
      : var(vname), kind(handlekind) {}

  void ToSpec(VarHandleSpec* spec) const {
    if (!name_space.empty()) spec->set_name_space(name_space);
    if (!var.empty()) spec->set_name(var);
    if (kind != Spec::UNK) spec->set_kind(kind);
  }

  bool operator<(const VarHandle& h2) const {
    if (kind < h2.kind) return true;
    if (kind > h2.kind) return false;
    if (name_space < h2.name_space) return true;
    if (name_space > h2.name_space) return false;
    if (var < h2.var) return true;
    if (var > h2.var) return false;
    return false;
  }

  //!
  //! Convenience constructor, creates a handle with kind "PARAM"
  static VarHandle ParamHandle(
      const string& vname_space, const string& vname) {
    return VarHandle(vname_space, vname, Spec::PARAM);
  }

  //!
  //! Convenience constructor, creates a handle with kind "PROP"
  static VarHandle PropertyHandle(
      const string& vname_space, const string& vname) {
    return VarHandle(vname_space, vname, Spec::PROP);
  }

  //!
  //! Returns true if fully specified to access global template values
  bool is_fully_specified() const { return !name_space.empty(); }

  //!
  //! Returns true if the handle specifies nothing.
  bool is_null() const {
    return kind == Spec::UNK && name_space.empty() && var.empty();
  }

  // TODO: remove the constant parsing.

  string full_name() const;

  //!
  //! Returns the last substring of var not containing a dot. E.g.
  //! for "a.b.c.d#1" short_name will return "d".
  string short_name() const;

  //!
  //! If the variable name is nested, returns all but the short_name.
  string name_prefix() const;

  //!
  //! Sets field num of the short name.
  void set_field_num(int i);

  //!
  //! Gets field num of the short name; -1 if no field number set.
  int get_field_num() const;

  HandleKind kind;
  string name_space;
  string var;
};

inline std::ostream& operator<<(std::ostream& os,
                                const VarHandle& handle) {
  os << handle.name_space << "." << handle.var << "["
     << VarHandleSpec::HandleKind_Name(handle.kind) << "]";
  return os;
}

// inline bool operator>(const VarHandle& h1, const VarHandle& h2) {
//   return h2 < h1;
// }

inline bool operator==(const VarHandle& h1, const VarHandle& h2) {
  return !(h1 < h2) && !(h2 < h1);
}

// TYPED PARAM------------------------------------------------------------------

//! Stores handle for a named variable along with its type information and any
//! other metadata.
class VarInfo {
 public:
  typedef VarInfoSpec::VarType VarType;

  static const string& VarTypeName(const VarType type);

  VarInfo() : handle_(), type_(VarInfoSpec::TYPE_UNK) {}

  VarInfo(const VarHandle& handle, const VarType& type)
      : handle_(handle), type_(type) {}

  VarInfo(const VarInfoSpec& spec)
      : handle_(spec.handle()), type_(spec.type()) {}

  void ToSpec(VarInfoSpec* spec) const {
    if (type_ != VarInfoSpec::TYPE_UNK) spec->set_type(type_);
    handle_.ToSpec(spec->mutable_handle());
  }

  const VarHandle handle() const { return handle_; }
  const string& name_space() const { return handle_.name_space; }
  const string& name() const { return handle_.var; }
  const VarHandle::HandleKind& kind() const { return handle_.kind; }

  const VarType& type() const { return type_; }

  void set_name_space(const string& vname_space) {
    handle_.name_space = vname_space;
  }

 private:
  // TODO: add pointer to the value
  VarHandle handle_;
  VarType type_;
};

inline std::ostream& operator<<(std::ostream& os,
                                const VarInfo& info) {
  os << info.handle() << " - " << VarInfo::VarTypeName(info.type());
  return os;
}

inline bool operator<(const VarInfo& info1, const VarInfo& info2) {
  return info1.handle() < info2.handle();
}

inline bool operator>(const VarInfo& info1, const VarInfo& info2) {
  return info2.handle() < info1.handle();
}

inline bool operator==(const VarInfo& info1, const VarInfo& info2) {
  return !(info1 < info2) && !(info2 < info1);
}


// NAMED VARIABLE CONTAINER ----------------------------------------------------

//! Super class for all containers providing controlled access to variables of
//! a limited set of types by class.
// Note: for each new type must implement:
// - virtual GetType = 0
// - virtual InternalSetType {return false;}
// - SetType() {boilerplate code}
class NamedVarContainer {
 public:
  NamedVarContainer() : dirty_(true) {}
  virtual ~NamedVarContainer() {}

  //!
  //! Returns true if some of the variable values have changed
  virtual bool is_dirty() const { return dirty_; }

  //!
  //! Marks all variables as clean
  virtual void make_clean() { dirty_ = false; }

  //!
  //! Returns top level variables of supported types
  virtual const set<VarInfo>& vars() const { return vars_; }

  //!
  //! Returns variables that are children of the provided variable
  //! (typically only works for protocol buffer types).
  virtual set<VarInfo> child_vars(const VarHandle& handle) const {
    return set<VarInfo>();
  }

  //!
  //! Returns true if contains no variables.
  bool empty() const { return vars().empty(); }

  //!
  //! Sets name space for all the VarHandles accessed by
  //! this container.
  //! To reimplemnt: reimplmenent set_name_space_internal.
  void set_name_space(const string& vname_space);

  //!
  //! Returns currently set namespace.
  const string& name_space() const { return name_space_; }

  //!
  //! Returns variable info for a given handle.
  virtual const VarInfo* GetVarInfo(const VarHandle& handle) const;

  //!
  //! Sets variable, should throw on error.
  //! Supports: int32, double, bool, const string&, const Message&.
  template <class T>
  bool Set(const VarHandle& handle, const T& value);

  //!
  //! Gets variable, should throw on error.
  //! Supports: int32, double, bool, const string&, const Message&.
  template <class T>
  T Get(const VarHandle& handle) const;

 protected:
  set<VarInfo> vars_;
  string name_space_;
  bool dirty_;

  virtual void set_name_space_internal(const string& vname_space);
  virtual bool SetInt(const VarHandle& handle, const int32& value) = 0;
  virtual bool SetBool(const VarHandle& handle, const bool& value) = 0;
  virtual bool SetDouble(const VarHandle& handle, const double& value) = 0;
  virtual bool SetString(const VarHandle& handle, const string& value) = 0;
  virtual bool SetMessage(const VarHandle& handle,
                          const google::protobuf::Message& value) = 0;

  virtual int32 GetInt(const VarHandle& handle) const = 0;
  virtual double GetDouble(const VarHandle& handle) const = 0;
  virtual bool GetBool(const VarHandle& handle) const = 0;
  virtual const string& GetString(const VarHandle& handle) const = 0;
  virtual const google::protobuf::Message& GetMessage(
      const VarHandle& handle) const = 0;

  struct HandleMatches {
    explicit HandleMatches(const VarHandle& handle) : handle_(handle) {}

    bool operator()(const VarInfo& info) {
      // match only by variable name
      return info.name() == handle_.var;
    }
   private:
    VarHandle handle_;
  };
};


// SIMPLE CONTAINER ------------------------------------------------------------

//! Allows storing variables by type
class SimpleContainer : public NamedVarContainer {
 public:
  SimpleContainer() {}

  bool IsInitialized(const VarHandle& handle) const;

  bool RemoveVariable(const VarHandle& handle);

  bool AddVariable(const VarInfo& info);

  template <class T>
  bool AddVariableUninit(const VarHandle& handle) {
    return AddVariableInternal<T>(handle, T());
  }

  template <class T>
  bool AddVariable(const VarHandle& handle, const T& value) {
    if (AddVariableInternal<T>(handle, value)) {
      initialized_.insert(handle);
      return true;
    }
    return false;
  }

  void PrintInfo() const;

 protected:
  virtual void set_name_space_internal(const string& vname_space);

  virtual bool SetInt(const VarHandle& handle, const int32& value);
  virtual bool SetBool(const VarHandle& handle, const bool& value);
  virtual bool SetDouble(const VarHandle& handle, const double& value);
  virtual bool SetString(const VarHandle& handle, const string& value);

  virtual bool SetMessage(const VarHandle& handle,
                          const google::protobuf::Message& value) {
    throw runtime_error("SimpleContainser::SetMessage not implemented.");
  }

  virtual int32 GetInt(const VarHandle& handle) const;
  virtual double GetDouble(const VarHandle& handle) const;
  virtual bool GetBool(const VarHandle& handle) const;
  virtual const string& GetString(const VarHandle& handle) const;

  virtual const google::protobuf::Message& GetMessage(
      const VarHandle& handle) const {
    throw runtime_error("SimpleContainser::GetMessage not implemented.");
  }

 private:
  template <class T>
  void SetNamespace(const string& vname_space) {
    std::map<VarHandle, T>& container = Container<T>();

    std::map<VarHandle, T> tmp_container;
    for (const auto& v : container) {
      VarHandle new_handle(v.first);
      new_handle.name_space = vname_space;
      tmp_container[new_handle] = v.second;
    }
    container = tmp_container;
  }

  template <class T>
  T GetInternal(const VarHandle& handle) const {
    const std::map<VarHandle, T>& container = Container<T>();
    typename std::map<VarHandle, T>::const_iterator it = container.find(handle);
    if (it == container.end()) {
      LOG(FATAL) << "Failed to get value for handle: " << handle;
    }
    return it->second;
  }

  template <class T>
  bool SetInternal(const VarHandle& handle, const T& value) {
    std::map<VarHandle, T>& container = Container<T>();
    typename std::map<VarHandle, T>::iterator it = container.find(handle);
    if (it == container.end()) {
      LOG(ERROR) << "Failed to get value for handle: " << handle;
      return false;
    }
    it->second = value;
    initialized_.insert(handle);
    return true;
  }


  template <class T>
  bool AddVariableInternal(const VarHandle& handle, const T& value) {
    if (GetVarInfo(handle)) {
      LOG(ERROR) << "Already has variable with handle: " << handle;
      return false;
    }

    std::map<VarHandle, T>& container = Container<T>();
    container[handle] = value;
    vars_.insert(VarInfo(handle, GetType<T>()));
    return true;
  }

  template <class T>
  const std::map<VarHandle, T>& Container() const;

  template <class T>
  std::map<VarHandle, T>& Container();

  template <class T>
  VarInfoSpec::VarType GetType() const;

  std::map<VarHandle, int32> ints_;
  std::map<VarHandle, bool> bools_;
  std::map<VarHandle, double> doubles_;
  std::map<VarHandle, string> strings_;
  std::set<VarHandle> initialized_;
};

// PROTO WRAPPER ---------------------------------------------------------------

//! Wraps a protocol buffer message that contains only fields that are supported
//! VarInfo types; allows access to parameters by name.
class ProtoMessageWrapper : public NamedVarContainer {
 public:
  explicit ProtoMessageWrapper(VarHandle::HandleKind kind = VarHandleSpec::UNK,
                               google::protobuf::Message* p = NULL);
  void Reset(google::protobuf::Message* p);
  virtual ~ProtoMessageWrapper() {}

  const google::protobuf::Message* proto() const { return proto_; }
  const string& proto_type() const;

  virtual set<VarInfo> child_vars(const VarHandle& handle) const;

  virtual const VarInfo* GetVarInfo(const VarHandle& handle) const;

 protected:
  virtual bool SetInt(const VarHandle& handle, const int32& value);
  virtual bool SetBool(const VarHandle& handle, const bool& value);
  virtual bool SetDouble(const VarHandle& handle, const double& value);
  virtual bool SetString(const VarHandle& handle, const string& value);
  virtual bool SetMessage(const VarHandle& handle,
                          const google::protobuf::Message& value);

  virtual int32 GetInt(const VarHandle& handle) const;
  virtual double GetDouble(const VarHandle& handle) const;
  virtual bool GetBool(const VarHandle& handle) const;
  virtual const string& GetString(const VarHandle& handle) const;
  virtual const google::protobuf::Message& GetMessage(
      const VarHandle& handle) const;

 private:
  template <class T>
  bool SetInternal(const VarHandle& handle, const T& value) {
    google::protobuf::Message* parent = GetDirectVarParentOrDie(handle);
    const google::protobuf::FieldDescriptor* fd =
        GetFieldDescriptorOrDie(handle.short_name(), parent);

    if (!SetParam<T>(parent, fd, handle.get_field_num(), value)) return false;
    return true;
  }

  template <class T>
  T GetInternal(const VarHandle& handle) const {
    const google::protobuf::Message* parent = GetDirectVarParentOrDie(handle);
    const google::protobuf::FieldDescriptor* fd =
        GetFieldDescriptorOrDie(handle.short_name(), parent);

    return GetParam<T>(parent, fd, handle.get_field_num());
  }

  template <class T>
  bool SetParam(google::protobuf::Message* direct_parent,
                const google::protobuf::FieldDescriptor* fd,
                const int field_num,
                const T& value);

  template <class T>
  T GetParam(const google::protobuf::Message* direct_parent,
             const google::protobuf::FieldDescriptor* fd,
             const int field_num) const;

  //! Supports parameter types:
  //!   * FieldDescriptor::CPPTYPE_INT32
  //!   * FieldDescriptor::CPPTYPE_DOUBLE
  //!   * FieldDescriptor::CPPTYPE_BOOL
  //!   * FieldDescriptor::CPPTYPE_STRING
  //!   * FieldDescriptor::CPPTYPE_MESSAGE
  bool CreateVarInfo(const google::protobuf::FieldDescriptor* fd,
                     VarInfo* info,
                     const string& prefix = "") const;

  // TODO: add support for more!!
  bool CreateVarInfoForMessage(const google::protobuf::FieldDescriptor* fd,
                               VarInfo* info,
                               const string& prefix = "") const;

  // Recursive
  void AddVarInfos(const google::protobuf::Message* direct_parent,
                   const google::protobuf::FieldDescriptor* fd,
                   set<VarInfo>* params,
                   const string& prefix = "") const;

  const google::protobuf::FieldDescriptor* GetFieldDescriptorOrDie(
      const string& name,
      const google::protobuf::Message* direct_parent) const;

  const google::protobuf::Message* GetDirectVarParentOrDie(
      const VarHandle& handle,
      size_t start_pos = 0) const {
    return GetDirectVarParentOrDie(handle, start_pos, proto_);
  }

  const google::protobuf::Message* GetDirectVarParentOrDie(
      const VarHandle& handle,
      size_t start_pos,
      const google::protobuf::Message* parent) const;

   google::protobuf::Message* GetDirectVarParentOrDie(
      const VarHandle& handle,
      size_t start_pos = 0) {
    return GetDirectVarParentOrDie(handle, start_pos, proto_);
  }

  google::protobuf::Message* GetDirectVarParentOrDie(
      const VarHandle& handle,
      size_t start_pos,
      google::protobuf::Message* parent);

  const VarHandle::HandleKind var_kind_;
  google::protobuf::Message* proto_;

  mutable std::map<VarHandle, VarInfo> var_info_cache_;
};


// UTILS -----------------------------------------------------------------------

bool WriteValueToSpec(const VarInfo* info,
                      const NamedVarContainer* container,
                      AnyValueSpec* spec);

bool SetValueFromSpec(const VarInfo* info,
                      NamedVarContainer* container,
                      const AnyValueSpec& spec);

template <class T>
T GetValueFromSpec(const AnyValueSpec& spec);

}  // namespace mit_plato

#endif  // _FAB_GEOMETRY_TEMPLATE_PARAMS_H__
