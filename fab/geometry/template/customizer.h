#ifndef _FAB_GEOMETRY_TEMPLATE_CUSTOMIZER_H__
#define _FAB_GEOMETRY_TEMPLATE_CUSTOMIZER_H__

#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <google/protobuf/stubs/common.h>  // for int32
#include <boost/function.hpp>
#include <boost/smart_ptr/scoped_ptr.hpp>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include "fab/geometry/template/params.h"
#include "fab/geometry/template/shape.h"
#include "fab/geometry/template/template.h"
#include "fab/geometry/template/template_params.h"
#include "fab/geometry/template/precomp/lookup.h"

namespace mit_plato {

using std::runtime_error;
using std::map;
using std::set;
using std::string;
using std::stringstream;
using std::vector;
using google::protobuf::int32;

class TemplateCustomizer;
class FabTemplateProto;
class FabGeometryTemplate;

//! Factory class for creating customizer from various encodings of
//! templates. Eventually may be extended to deal with instantiating
//! a customizer for a template with pre-computed geometry stored in
//! a db.
//!
//! Call as follows:
//! boost::scoped_ptr<TemplateCustomizer> customizer(
//!   CustomizerFactory::From{File,Spec,etc...}(...));
//! if (customizer == NULL) {
//!   cerr << "Error!";
//!   return 1;
//! }
//!
//! customizer->Get...();
class CustomizerFactory {
 public:
  // If enabled, cached preloaded resources (expensive to load)
  // based on customizer filename or cache_key.
  static void SetResourceCaching(bool enable);

  // If disabled, full cache is used, and optimized keys are ignored;
  // this is enabled by default.
  static void SetCacheOptimization(bool enable);

  //! Instantiates a customizer encoded in CustomizerSpec; picks customizer
  //! subclass based on the fields present in the spec.
  //! If the file is FabTemplateProto, returns a simple customizer.
  static TemplateCustomizer* FromFile(const string& spec);

  //! Instantiates a customizer for a template, picks a subclass based on
  //! the fields present in the spec.
  static TemplateCustomizer* FromSpec(const CustomizerSpec& spec,
                                      const string& cache_key = "");

  //! Instantiate a customizer for a template encoded in FabTemplateProto spec.
  //! Returns NULL on error; caller responsible for deallocating customizer.
  static TemplateCustomizer* FromTemplateSpec(const FabTemplateProto& spec);

  //! Debug version of customizer; do not use in production.
  static TemplateCustomizer* DebugCustomizer(
      boost::shared_ptr<FabGeometryTemplate> tpl);

  // Absolutizes relative paths based on the flag value of the precomp directory
  static void AbsolutizePaths(const CustomizerSpec& spec);

  static CustomizerFactory& Singleton();

  GeometryLookup* GetGeometryLookup(const CustomizerSpec& spec,
                                    const string& cache_key);

  PrecompLookup* GetPrecompLookup(const CustomizerSpec& spec,
                                  const FabGeometryTemplate* tpl,
                                  const string& cache_key);
 private:
  CustomizerFactory()
      : cache_resources_(false),
        disable_cache_optimization_(false) {}
  ~CustomizerFactory();
  void ReleaseResources();

  bool cache_resources_;
  bool disable_cache_optimization_;
  std::map<string, PrecompLookup*> lookups_;
  std::map<string, GeometryLookup*> geo_lookups_;
  boost::mutex geo_lock_;
  boost::mutex lookup_lock_;
};


//!
//! THE ONLY INTERFACE TO BE USED FOR CUSTOMIZING A TEMPLATE.
//! Create an instance of the customizer by calling a method of the
//! CustomizerFactory.
//! The class must ensure that only valid geometry results from user
//! manipulation of the parameters.
//!
//! (Note: to edit the actual template specification, use TemplateEditor.)
class TemplateCustomizer {
 public:
  friend class CustomizerFactory;

  enum Status {UPDATE_FAILED, UPDATE_SUCCEEDED, PREVIEW_SUCCEEDED, PREVIEW_FAILED};
  typedef boost::function<void (Status status, TemplateCustomizer* c)> CallbackFunc;

  virtual ~TemplateCustomizer() {}

  // Customizer Options --------------------------------------------------------

  void set_snap_enabled(bool enable = true) { snap_enabled_ = enable; }

  bool snap_enabled() const { return snap_enabled_; }

  // User Controls Metadata ----------------------------------------------------

  //!
  //! Returns the name of the template if it has been set.
  const string& name() const { return tpl_->name(); }

  //! Returns a full list of available controls, with information on control
  //! name, its handle, type and bounds.
  //! These handles can be used to call Get and Set methods on the variables,
  //! appropriate to their task.
  const vector<TemplateParams::ControlInfo>& Controls() const;

  //! Returns encoded seeds for easier exploration of the valid region.
  const vector<string>& seeds() const {return seeds_;}

  //! Returns information for a particular variable by its handle;
  //! returns NULL if variable is not found.
  const VarInfo* GetVarInfo(const VarHandle& handle) const;

  TemplateParams::ControlInfo GetControlInfo(const string& name) const;

  // Value Getters -------------------------------------------------------------

  //! Returns parameter values by handle (fatal failure may result if
  //! wrong method is called for the variable type).
  int32 GetValueInt(const VarHandle& handle) const;

  double GetValueDouble(const VarHandle& handle) const;

  bool GetValueBool(const VarHandle& handle) const;

  const string& GetValueString(const VarHandle& handle) const;

  const VectorParam& GetValueVector(const VarHandle& handle) const;

  // Bound Getters -------------------------------------------------------------

  virtual const IntBounds& GetBoundsInt(const VarHandle& handle) const;

  virtual const DoubleBounds& GetBoundsDouble(const VarHandle& handle) const;

  // Note: no bool bounds, bool only has 2 values

  // TODO: implement GetBounds for vectors

  // Value Setters -------------------------------------------------------------

  // Jumps to a given seed value
  virtual bool JumpToSeed(const string& seed) { return false; };

  //! Returns true if the value was set successfully. Should call controls to
  //! get updated bound/value information.
  virtual bool SetValueInt(
      const VarHandle& handle,
      const int32& value) = 0;

  virtual bool SetValueDouble(
      const VarHandle& handle,
      const double& value) = 0;

   virtual bool SetValueBool(
       const VarHandle& handle,
       const bool& value) = 0;

   virtual bool SetValueString(
       const VarHandle& handle,
       const string& value) = 0;

   virtual bool SetValueVector(
       const VarHandle& handle,
       const VectorParam& value) = 0;

  // Geometry Updates ----------------------------------------------------------

  //! High-level function, may result in callback being called more than once.
  //! For example,
  void RequestGeometryUpdate(CallbackFunc callback);

  //! Get geometry after an update
  virtual const vector<boost::shared_ptr<const Shape> >& GetGeometry() const = 0;

  // Geometry Updates (low-level functions) ------------------------------------

  //! Updates geometry sequentially.
  Status UpdateGeometry();

  //! Updates geometry in the background, returns fast.
  //! When the update is done, will call callback with
  //! - true if update succeeded; false o/w
  //! - a pointer to this customizer instance
  //! Note: no need to use the returned thread object; it can be ignored.
  boost::thread UpdateGeometryAsync(CallbackFunc callback);

  // Misc ----------------------------------------------------------------------

  //! Writes the value of control for a set of supported types to spec;
  //! returns true on success.
  bool ValueToSpec(const VarHandle& control_handle,
                   AnyValueSpec* spec) const;

  const LookupStatsProto& stats_proto() const { return stats_spec_; }

  boost::shared_ptr<FabGeometryTemplate> debug_tpl() {
    return tpl_;
  }

 protected:
  TemplateCustomizer();
  TemplateCustomizer(boost::shared_ptr<FabGeometryTemplate> tpl);
  void Init();

  TemplateCustomizer::Status UpdateGeometryUnlocked();

  vector<string> seeds_;

  virtual Status UpdateGeometryImpl() = 0;

  template <class B>
  const B& GetBounds(const VarHandle& handle) const;

  boost::shared_ptr<FabGeometryTemplate> tpl_;
  vector<TemplateParams::ControlInfo> controls_;

  vector<boost::shared_ptr<const Shape> > preview_;

 private:
  void UpdateGeometryTask(CallbackFunc callback);

  boost::mutex lock_;
  CallbackFunc pending_callback_;

  boost::mutex lock_state_;
  bool geometry_update_pending_;
  bool geometry_update_running_;

  // Options
  bool snap_enabled_;

  // Stats spec
  LookupStatsProto stats_spec_;
};

template <class B>
const B& TemplateCustomizer::GetBounds(const VarHandle& handle) const {
  const B* bounds =
      dynamic_cast<const B*>(
          tpl_->params_container().GetMutableVarBounds(handle));
  if (!bounds) {
    stringstream ss;
    ss << "Could not get typed bounds for: " << handle;
    throw runtime_error(ss.str());
  }
  return *bounds;
}

// CUSTOMIZER IMPLEMENTATIONS --------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
// SOME CODE REMOVED
////////////////////////////////////////////////////////////////////////////////

}  // namespace mit_plato

#endif  // _FAB_GEOMETRY_TEMPLATE_CUSTOMIZER_H__
