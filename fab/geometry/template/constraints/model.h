#ifndef _FAB_GEOMETRY_TEMPLATE_CONSTRAINTS_MODEL_H__
#define _FAB_GEOMETRY_TEMPLATE_CONSTRAINTS_MODEL_H__

#include <boost/smart_ptr/scoped_ptr.hpp>
#include <gecode/float.hh>
#include <gecode/int.hh>

#include "fab/geometry/template/model_helper.h"


namespace mit_plato {

class FabGeometryTemplate;

class TemplateModel : public Gecode::Space {
 public:
  TemplateModel(const FabGeometryTemplate* tpl);

  TemplateModel(bool share, TemplateModel& m);

  virtual Space* copy(bool share);

  void SetSolution(FabGeometryTemplate* tpl) const;

 private:
  //  BoolVarArray bool_vars_;
  Gecode::IntVarArray int_vars_;
  Gecode::FloatVarArray double_vars_;

  boost::scoped_ptr<TemplateModelHelper> helper_;
};

}  // namespace mit_plato

#endif  // _FAB_GEOMETRY_TEMPLATE_CONSTRAINTS_MODEL_H__
