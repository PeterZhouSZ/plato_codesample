#ifndef _FAB_GEOMETRY_TEMPLATE_CONSTRAINTS_MODEL_HELPER_H__
#define _FAB_GEOMETRY_TEMPLATE_CONSTRAINTS_MODEL_HELPER_H__

#include <map>
using std::map;

#include <gecode/float.hh>
#include <gecode/int.hh>

#include "fab/geometry/template/params.h"
#include "fab/geometry/template/template_params.h"


namespace mit_plato {

class TemplateModel;

//!
//! Helper class that keeps association between names and indices.
class HandleIndexer {
 public:
  void Insert(const VarHandle& h, int i);

  bool HasVar(const VarHandle& h) const;

  const VarHandle& Handle(const int i) const;

  const int Index(const VarHandle& h) const;

  const map<int, VarHandle>& index_map() const { return index_map_; }

 private:
  map<VarHandle, int> handle_map_;
  map<int, VarHandle> index_map_;
};


//!
//! Helper class for constructing model variables from template and keeping
//! track of association between variable values in model and variable handles
//! in the template.
class TemplateModelHelper {
 public:
  TemplateModelHelper(Gecode::Space& home, const TemplateParams* params);

  TemplateModelHelper(const TemplateModelHelper& helper);

  bool HasVar(const VarHandle& handle) const;

  const Gecode::IntVar& GetVarInt(const VarHandle& name) const;

  const Gecode::FloatVar& GetVarDouble(const VarHandle& name) const;

  Gecode::Space& home() { return home_; }

  // Support for auxiliary variables -------------------------------------------
  VarHandle AuxDoubleHandle(const string& var_name) const;

  const Gecode::FloatVar& NewAuxDouble(
      const VarHandle& handle, double min_val, double max_val);

 private:
  VarHandle GenerateNewHandle() const;
  bool AddNewHandle(const VarHandle& h);

  set<VarHandle> ids_;

  HandleIndexer int_ids_;
  Gecode::IntVarArgs int_args_;

  HandleIndexer aux_double_ids_;
  HandleIndexer double_ids_;
  Gecode::FloatVarArgs double_args_;

  // TODO: doubles, bools, etc.

  Gecode::Space& home_;
  const TemplateParams* params_;

  friend class TemplateModel;
};


}  // namespace mit_plato

#endif  // _FAB_GEOMETRY_TEMPLATE_CONSTRAINTS_MODEL_HELPER_H__
