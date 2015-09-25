#ifndef _FAB_GEOMETRY_TEMPLATE_TEMPLATE_SAMPLING_MAPPER_H__
#define _FAB_GEOMETRY_TEMPLATE_TEMPLATE_SAMPLING_MAPPER_H__

#include <string>
#include <set>
#include <vector>
#include <fstream>

#include "fab/geometry/template/precomp/key_util.h"
#include "fab/geometry/tri_mesh.h"

namespace mit_plato {

class FabGeometryTemplate;
class ControlValues;
class PrecompLookup;
class Hypercuboid;
class GeometryLookup;

using std::string;
using std::set;

// Proceses a sample (in the design space) for a particular
// geometry template; outputs all results to files
class SamplingMapper {
 public:

  // Instantiated once per remote worker.
  //
  // control_names: csv identifies of user-visible controls
  // tpl: instantiation of the corresponding template
  // saples_stream: stream to output sample validity/properties to
  // geometry_stream: stream to output metadata for liberally cached geometry subparts
  // local_geometry_output_dir: where to store cached geometry locally (on the worker machine)
  // aws_geometry_output_dir: where to store geometry on S3
  SamplingMapper(const string& control_names,
                 FabGeometryTemplate* tpl,
                 std::ofstream& samples_stream,
                 std::ofstream& geometry_stream,
                 const string& local_geometry_output_dir,
                 const string& aws_geometry_output_dir);

  void SetLookup(PrecompLookup* lookup);

  void ProcessSample(const string& control_values_str);

  void ProcessSample(const ControlValues& control_values);

  void ProcessSamples(const vector<const ControlValues*>& samples);

  // Skip samples that have already been computed
  void SkipProcessedSamples(const string& samples_file);

  // Init geometry cache with existing cache
  void InitializeGeometryCache(const string& geo_index_file);

  // Get subdivision dimensions sorted preferentially
  void GetSubdivisionDimensions(const Hypercuboid& cuboid,
                                PrecompLookup* lookup,
                                std::vector<int>* dims);

  const GeometryLookup& geo_cache() const { return geo_cache_; }

 private:
  void ProcessSampleInternal(const string& file_suffix,
                             const ControlValues* sample = NULL);

  bool CacheMesh(const TriMesh& mesh,
                 SubtreeCachingInfo* info);

  void WriteCachingInfo(const SubtreeCachingInfo& info);

  KeyUtil key_util_;
  FabGeometryTemplate* tpl_;
  GeometryLookup geo_cache_;
  PrecompLookup* lookup_;

  std::ofstream& samples_stream_;
  std::ofstream& geo_index_stream_;
  const string local_geo_dir_;
  const string aws_geo_dir_;

  set<string> computed_keys_;
};

}  // namespace mit_plato

#endif  // _FAB_GEOMETRY_TEMPLATE_TEMPLATE_SAMPLING_MAPPER_H__
