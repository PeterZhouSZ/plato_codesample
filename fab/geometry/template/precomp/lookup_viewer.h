#ifndef _FAB_GEOMETRY_TEMPLATE_TEMPLATE_PRECOMP_LOOKUP_VIEWER_H__
#define _FAB_GEOMETRY_TEMPLATE_TEMPLATE_PRECOMP_LOOKUP_VIEWER_H__

#include <list>

#include <boost/date_time/posix_time/posix_time_types.hpp>

#include <QEvent>
#include <QGLViewer/qglviewer.h>
#include <QKeyEvent>
#include <QMouseEvent>

#include "fab/geometry/template/customizer.h"
#include "fab/geometry/template/precomp/lookup.h"
#include "fab/geometry/template/precomp/lookup_renderer.h"
#include "fab/geometry/template/smart_customizer.h"

namespace mit_plato {

class LookupStats;

// Interactive visualization of the precomputed design space. Allows:
// - jumping to partiular point
// - visualizing valid intervals
// - slicing through the space along a given dimension (video)
// - visualizing a subset of the dimensions (for dimensionality > 3)
class LookupViewer : public QGLViewer {
 public:
  enum ViewMode { VALIDITY = 0,
                  DELTA_GEO = 1,
                  SAMPLES = 2,
                  VALID_SAMPLES = 3,
                  PROPERTY = 4,
                  REGIONS = 5};

  enum PropertyMode { BAD_CONSTRAINT = 0, BAD_GEOMETRY = 1, NOT_STABLE = 2,
                      VOLUME = 3, N_CACHED_SUBTREES = 4, CACHE_SPACE = 5 };

  LookupViewer(TemplateCustomizer& customizer);
  virtual ~LookupViewer();

  virtual void init();
  virtual void draw();
  virtual bool eventFilter(QObject *Object, QEvent *Event);
  void print_info() const;

  const ControlValues& CurrentControlValues() const;
  void OnGeometryUpdate(TemplateCustomizer::Status status,
                        TemplateCustomizer* c);
  string GetControlsText(bool print_type = false) const;

  void animate();

 private:
  const TemplateParams::ControlInfo* GetControlInfoCmd(int* index) const;
  void SetValueToCut(const int dim_index,
                     const TemplateParams::ControlInfo* cinfo,
                     const int cut_index);
  void SliceThroughDimension();

  void PrintConnectedComponents() const;
  void PrintControls() const;
  void PrintBounds() const;
  void ModifyControls();
  void SetDisplayAxes(std::istream& input);

  double GetTimeMs() const;
  void PlayScriptedVideo();
  void ExecuteCommand(const string& command, const string& args);

  SmartCustomizer& customizer_;
  const PrecompLookup& lookup_;

  // View settings
  ViewMode mode_;
  bool draw_sample_;
  LookupRenderer renderer_;
  mutable LookupStats* stats_;
  int active_component_;

  // For animation
  std::list<std::pair<double, std::pair<string, string> > > commands_;
  std::pair<int, const TemplateParams::ControlInfo*> slice_dimension_;
  int current_cut_;
  boost::posix_time::ptime start_time_;
};

}  // namespace mit_plato

#endif  // _FAB_GEOMETRY_TEMPLATE_TEMPLATE_PRECOMP_LOOKUP_VIEWER_H__
