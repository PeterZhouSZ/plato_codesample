#ifndef _FAB_GEOMETRY_TEMPLATE_TEMPLATE_PARAMS_H__
#define _FAB_GEOMETRY_TEMPLATE_TEMPLATE_PARAMS_H__

#include <map>
#include <string>
using std::string;

#include <glog/logging.h>

#include "fab/geometry/template/params.h"
#include "fab/geometry/template/bounds.h"


namespace mit_plato {

class Operation;


//! Simple collection of all of the Template's parameters;
//! does not hold any knowledge about constraints, parameter
//! interdependencies or underlying operations.
//! Holds information about additional metavariables created
//! in the environment and about parameters that are allowed to
//! vary and their ranges.
class TemplateParams : public NamedVarContainer {
 public:
  struct ControlInfo;

  virtual ~TemplateParams();

  // I/O -----------------------------------------------------------------------

  // Writes mutables, metavars, etc. to spec
  bool ToSpec(FabTemplateProto* spec) const;

  bool InitFromSpec(const FabTemplateProto& spec);

  // Control support -----------------------------------------------------------

  //! Marks a mutable variable as a control
  bool MarkAsControl(const VarHandle& vh, const string& name);

  bool RemoveControl(const VarHandle& vh);

  const string& ControlName(const VarHandle& vh) const;

  const VarHandle* ControlHandle(const string& name) const;

  const std::map<VarHandle, string>& controls() const;

  // Same as controls(), but gets additional information
  void controls(std::vector<ControlInfo>* controls) const;

  ControlInfo GetControlInfo(const string& name) const;

  bool IsControl(const VarHandle& vh) const;

  // Mutable vars & bounds -----------------------------------------------------

  // Takes ownership of bounds
  bool MarkAsMutableInt(const VarHandle& handle,
                        IntBounds* bounds);

  bool MarkAsMutableDouble(const VarHandle& handle,
                           DoubleBounds* bounds);

  bool MarkAsMutableBool(const VarHandle& handle);

  bool MarkAsImmutable(const VarHandle& handle);

  bool IsMutable(const VarHandle& handle) const;

  bool IsChildMutable(const VarHandle& handle) const;

  // returns NULL if none found
  const Bounds* GetMutableVarBounds(const VarHandle& handle) const;

  const std::map<VarInfo, Bounds*>& mutable_vars() const;

  // Auxiliary variable support ------------------------------------------------

  bool CreateAuxVariable(const VarInfo& info);

  bool DeleteAuxVariable(const VarHandle& handle);

  // Metavariable support ------------------------------------------------------

  // Simply creates a variable of a given type;
  // call Set<type> to set it to a particular value;
  // call MarkAsMutable to set the bounds.
  bool CreateMetaVariable(const string& name,
                          const VarInfoSpec::VarType type,
                          VarHandle* handle);

  bool DeleteMetaVariable(const VarHandle& handle);

  const set<VarInfo>& meta_vars() const;

  // Basic NamedVarContainer interface -----------------------------------------

  //!
  //! Returns true if any operation's params are dirty.
  virtual bool is_dirty() const;

  //!
  //! Marks all operations as clean.
  virtual void make_clean();

  //!
  //! Returns child vars for a given variable, or empty.
  virtual set<VarInfo> child_vars(const VarHandle& handle) const;

  //!
  //! Returns metadata about a given parameter / property.
  virtual const VarInfo* GetVarInfo(const VarHandle& handle) const;

  //!
  //! Adds parameters from OperationNode to the collection.
  void AddOperationParams(const string& name,
                          Operation* operation);

  //!
  //! Removes parameters for operation with name.
  void RemoveOperationParams(const string& name);

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

  struct ControlInfo {
    ControlInfo(const string& in_name,
                const VarInfo* in_info,
                const Bounds* in_bounds,
                const Bounds* in_outer_bounds)
        : name_(in_name),
          info_(NULL),
          bounds_(NULL),
          outer_bounds_(NULL) {
      Reset(in_name, in_info, in_bounds, in_outer_bounds);
    }

    ControlInfo(const ControlInfo& other)
        : name_(other.name()),
          info_(NULL),
          bounds_(NULL),
          outer_bounds_(NULL) {
      Reset(other.name_, other.info_, other.bounds_, other.outer_bounds_);
    }

    ControlInfo& operator=(const ControlInfo& other) {
      Reset(other.name_, other.info_, other.bounds_, other.outer_bounds_);
      return *this;
    }

    ~ControlInfo() {
      delete info_;
      delete bounds_;
      delete outer_bounds_;
    }

    const VarInfo* info() const {
      return info_;
    }

    const string& name() const {
      return name_;
    }

    const Bounds* bounds() const {
      return bounds_;
    }

    void set_bounds(const Bounds* b) {
      delete bounds_;
      if (b) {
        bounds_ = b->Copy();
      } else {
        bounds_ = NULL;
      }
    }

    void set_outer_bounds(const Bounds* b) {
      delete outer_bounds_;
      if (b) {
        outer_bounds_ = b->Copy();
      } else {
        outer_bounds_ = NULL;
      }
    }

   private:
    void Reset(const string& in_name,
               const VarInfo* in_info,
               const Bounds* in_bounds,
               const Bounds* in_outer_bounds) {
      delete info_;
      delete bounds_;
      delete outer_bounds_;

      name_ = in_name;

      if (in_info) {
        info_ = new VarInfo(*in_info);
      } else {
        info_ = NULL;
      }

      set_bounds(in_bounds);
      set_outer_bounds(in_outer_bounds);
    }

    string name_;
    const VarInfo* info_;
    const Bounds* bounds_;
    const Bounds* outer_bounds_;

    ControlInfo()
        : info_(NULL), bounds_(NULL), outer_bounds_(NULL) {}  // disallow
  };

 private:
  //!
  //! Never call; causes the program to crash.
  virtual void set_name_space_internal(const string& vname_space);

  //!
  //! Given fully specified parameter handle, sets its value;
  //! only implemented for types that are valid TypedParams.
  template <class T>
  bool SetInternal(const VarHandle& handle, const T& value) {
    VLOG(3) << "Setting " << handle;
    if (handle.kind == VarHandleSpec::AUX) {
      return aux_container_.Set<T>(handle, value);
    } else if (handle.kind == VarHandleSpec::META) {
      return meta_container_.Set<T>(handle, value);
    } else if (handle.is_fully_specified()) {
      return CHECK_NOTNULL(FindWrapper(handle.name_space))
          ->Set<T>(handle, value);
    } else {
      LOG(ERROR) << "Failed to set " << handle;
      return false;
    }
  }

  //!
  //! Given fully specified parameter handle, gets its value;
  //! only implemented for types that are valid TypedParams.
  template <class T>
  T GetInternal(const VarHandle& handle) const {
    if (handle.kind == VarHandleSpec::AUX) {
      return aux_container_.Get<T>(handle);
    } else if (handle.kind == VarHandleSpec::META) {
      return meta_container_.Get<T>(handle);
    } else if (handle.is_fully_specified()) {
      const NamedVarContainer* wrapper = FindWrapper(handle.name_space);
      CHECK(wrapper) << "Failed to find wrapper for handle " << handle.name_space;
      return wrapper->Get<T>(handle);
    } else {
      throw runtime_error("Cannot get var: " + handle.full_name());
    }
  }

  bool MarkAsMutable(const VarHandle& handle,
                     Bounds* bounds,
                     VarInfoSpec::VarType expected_type);

  // Resets internal vars_ from all operation_params_
  void ResetVars();

  NamedVarContainer* FindWrapper(const string& op_name);
  const NamedVarContainer* FindWrapper(const string& op_name) const;

  std::map<string, NamedVarContainer*> operation_params_;
  SimpleContainer meta_container_;
  SimpleContainer aux_container_;
  std::map<VarInfo, Bounds*> mutables_;
  std::map<VarHandle, string> controls_;  // and their names
};

}  // namespace mit_plato

#endif // _FAB_GEOMETRY_TEMPLATE_TEMPLATE_PARAMS_H__
