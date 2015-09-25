#ifndef _FAB_GEOMETRY_TEMPLATE_TEMPLATE_PRECOMP_LOOKUP_RENDERER_H__
#define _FAB_GEOMETRY_TEMPLATE_TEMPLATE_PRECOMP_LOOKUP_RENDERER_H__

#include <vector>
using std::vector;

#include <Eigen/Dense>
#include "fab/geometry/template/precomp/lookup.h"
#include "fab/geometry/template/precomp/stats.h"
#include "util/color_convert.h"

namespace mit_plato {

class LookupStats;

class LookupRenderer {
 public:
  enum ColorModulation {
     SOLID_COLOR = 0, FALSE_Z_SHADOW = 1, EDGE_WEIGHT_COLOR = 2};
  LookupRenderer(const PrecompLookup* lookup);
  ~LookupRenderer();

  const PrecompLookup* lookup() const { return lookup_; }
  const vector<int>& dimentions() const { return dims_; }
  void SetViewDimentions(const vector<int>& dims);
  Sample* current_sample() { return &sample_; }
  const Sample* current_sample() const { return &sample_; }

  void set_render_properties(bool val) { render_properties_ = val; }

  void RenderSamples(bool only_valid = false) const;
  void RenderCurrentSample() const;
  void RenderBounds() const;
  void RenderValidity() const;
  void RenderDeltaGeo(LookupStats* stats) const;
  void RenderGraphRegions(LookupStats* stats, int active_comp = -1) const;
  //void RenderProperty() const;

  void RenderValidity(const Hypercuboid& hc) const;
  void RenderProperties(const Hypercuboid& hc) const;
//  void RenderDeltaGeo(const Hypercuboid& hc) const;

  const vector<const Hypercuboid*>& leaves() const { return leaves_; }

 private:
  double ToViewCoord(const double val, const int dim) const;
  Eigen::Vector3d ToViewCoords(const Sample& s) const;
  Eigen::Vector3d ToViewCoords(const vector<double>& sample_vector) const;

  void RenderInterval(const int d, vector<double> tmp_values,
                      double imin, double imax) const;

  void RenderGraph(const LookupStats::GraphType& graph,
                   mds_util::Hsv color,
                   const Sample* center_sample = NULL,
                   ColorModulation color_mode = SOLID_COLOR) const;

  void RenderSample(const Sample& s,
                    bool highlighted = false,
                    bool auto_color = true) const;

  bool ShouldRenderCuboid(const Hypercuboid& hc) const;
  bool ShouldRenderSample(const Sample& s) const;

  void glHypercuboidVertex(
      const Hypercuboid& hc, int i, int j, int k) const;
  const Sample* GetCuboidVertex(
      const Hypercuboid& hc, int i, int j, int k) const;
  void SetVertexColor(const Sample* s) const;


  vector<int> dims_;
  bool render_properties_;

  const PrecompLookup* lookup_;
  vector<const Sample*> samples_;
  vector<const Hypercuboid*> leaves_; // should be deleted

  Sample sample_;
};

}  // namespace mit_plato

#endif  // _FAB_GEOMETRY_TEMPLATE_TEMPLATE_PRECOMP_LOOKUP_RENDERER_H__
