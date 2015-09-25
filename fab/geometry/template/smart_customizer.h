#ifndef _FAB_GEOMETRY_TEMPLATE_SMART_CUSTOMIZER_H__
#define _FAB_GEOMETRY_TEMPLATE_SMART_CUSTOMIZER_H__

#include <glog/logging.h>

#include "fab/geometry/template/customizer.h"
#include "fab/geometry/template/precomp/lookup.h"

namespace mit_plato {

//!
//! Customizer supporting pre-computation, geometry caching,
//! smart parameter bounds, etc. Actual capabilities depend on
//! the lookup and cache files it is instantiated with.
class SmartCustomizer : public TemplateCustomizer {
 public:
  friend class CustomizerFactory;

  ~SmartCustomizer();

  // Value Setters -------------------------------------------------------------

  virtual bool JumpToSeed(const string& seed);

  virtual bool SetValueInt(const VarHandle& handle,
                           const int32& value);

  virtual bool SetValueDouble(const VarHandle& handle,
                              const double& value);

  virtual bool SetValueBool(const VarHandle& handle,
                            const bool& value);

  virtual bool SetValueString(const VarHandle& handle,
                              const string& value);

  virtual bool SetValueVector(const VarHandle& handle,
                              const VectorParam& value);

  virtual const std::vector<boost::shared_ptr<const Shape> >& GetGeometry() const;

  const PrecompLookup* DebugLookup() const { return lookup_; }
  const FabGeometryTemplate* DebugTpl() const { return tpl_.get(); }

 protected:
  virtual Status UpdateGeometryImpl();

  bool MaybeSnap();
  bool UpdateControls();

 private:
  template <class T>
  bool SetValueInternal(const VarHandle& handle,
                        const T& val);

  // Use CustomizerFactory to instantiate all customizers
  SmartCustomizer() : lookup_(NULL), cache_(NULL) {}

  SmartCustomizer(boost::shared_ptr<FabGeometryTemplate> tpl,
                  PrecompLookup* lookup = NULL,
                  GeometryLookup* geo_cache = NULL);

  FabGeometryTemplate* active_tpl();

  // Data
  boost::scoped_ptr<FabGeometryTemplate> tpl_copy_;
  PrecompLookup* lookup_;
  GeometryLookup* cache_;

  boost::mutex controls_lock_;
  vector<Bounds*> current_bounds_;
};

template <class T>
bool SmartCustomizer::SetValueInternal(const VarHandle& handle,
                                       const T& value) {
  if (KeyUtil::AreTwoEqual<T>(value, active_tpl()->GetParam<T>(handle))) {
    LOG(INFO) << "New value almost identical: " << handle
              << " <-- " << value << " ===> SKIPPING";
    return false;
  }

  LOG(INFO) << "Setting " << handle << " <-- " << value;
  boost::mutex::scoped_lock lock(controls_lock_);
  if (!active_tpl()->SetParam(handle, value)) { return false; }

  return UpdateControls();
}


}  // namespace mit_plato

#endif
