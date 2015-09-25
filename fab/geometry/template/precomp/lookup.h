#ifndef _FAB_GEOMETRY_TEMPLATE_TEMPLATE_PRECOMP_LOOKUP_H__
#define _FAB_GEOMETRY_TEMPLATE_TEMPLATE_PRECOMP_LOOKUP_H__

#include <ostream>
#include <string>
#include <map>
#include <vector>
#include <utility>
using std::string;
using std::map;
using std::vector;

#include <boost/smart_ptr/shared_ptr.hpp>
#include <boost/smart_ptr/scoped_ptr.hpp>

#include <Eigen/Dense>

#include "fab/geometry/tri_mesh.h"
#include "fab/geometry/template/template.pb.h"
#include "fab/geometry/template/template.h"
#include "fab/geometry/template/precomp/key_util.h"
#include "fab/geometry/template/precomp/geometry_cache.h"

namespace mit_plato {

using google::int64;

class Shape;
class FabGeometryTemplate;
class TemplateParams;

class Dimension;
class ControlValues;
typedef ControlValues Sample;
class ControlValuesHelper;


// GEOMETRY DELTA LOOKUP -------------------------------------------------------

class ValidityGrid2D {
 public:
  ValidityGrid2D(int x_dim, int y_dim)
      : x_dim_(x_dim), y_dim_(y_dim) {
    positions_.resize(x_dim * y_dim * 2, 0);
    failure_codes_.resize(x_dim * y_dim, 0);
  }

  void SetPoint(int i, int j,
                float x, float y,
                int failure_code = 0) {
    int index = j * x_dim_ + i;
    positions_[index * 2] = x;
    positions_[index * 2 + 1] = y;
    failure_codes_[index] = failure_code;
  }

  int x_dim() const {
    return x_dim_;
  }

  int y_dim() const {
    return y_dim_;
  }

  vector<float> positions_;
  vector<short int> failure_codes_;

 private:
  int x_dim_;
  int y_dim_;
};

// Cache for the values of change in geometry between various sample pairs
class GeometryDeltaLookup {
 public:
  GeometryDeltaLookup();

  // Enables computing a delta if it does not exist
  void EnableComputation(const GeometryLookup* geo_cache);

  void EnableBackup(std::ofstream* backup) { backup_stream_ = backup; }

  void SetDelta(const Sample* s1, const Sample* s2, double value);

  bool HasDelta(const Sample* s1, const Sample* s2) const;

  double GetGeometryDelta(const Sample* s1, const Sample* s2) const;

  void CacheGeometryDelta(const ControlValuesHelper& helper,
                          const Sample* s1, const Sample* s2);

  const map<std::pair<const Sample*, const Sample*>, double>& cache() const {
    return cache_;
  }

  void ToSpec(const ControlValuesHelper& helper, GeoDeltasSpec* spec) const;

  void LogInfo() const;

 private:
  Shape* GetShape(const ControlValuesHelper& helper, const Sample* s);

  void ToSpec(const ControlValuesHelper& helper,
              const std::pair<std::pair<const Sample*, const Sample*>, double>& entry,
              GeoDeltasSpec::Entry* dspec) const;

  void BackupDelta(
      const ControlValuesHelper& helper,
      const std::pair<std::pair<const Sample*, const Sample*>, double>& entry) const;

  const GeometryLookup* geo_cache_;
  map<std::pair<const Sample*, const Sample*>, double> cache_;

  // Backup_file
  std::ofstream* backup_stream_;
};

// CONTROL VALUES --------------------------------------------------------------

class ControlValuesHelper;
class ControlValuesThreshold;

// Can encode control values for an arbitrary geometry template, regardless of
// the type and number of end-user-visible controls.
class ControlValues {
 public:
  ControlValues() {}

  ControlValues(const ControlValuesHelper& helper);

  ControlValues(const ControlValuesHelper& helper,
                const string& encoded);

  ControlValues(const ControlValuesHelper& helper,
                const FabGeometryTemplate* tpl);

  ControlValues(const ControlValues& other)
      : doubles_(other.doubles_),
        ints_(other.ints_),
        bools_(other.bools_) {}

  void ToDoubleVector(vector<double>* values) const {
    values->insert(values->end(), doubles().begin(), doubles().end());
    values->insert(values->end(), ints().begin(), ints().end());
    values->insert(values->end(), bools().begin(), bools().end());
  }

  bool is_empty() const {
    return ints_.empty() && bools_.empty() && doubles_.empty();
  }

  int size() const {
    return ints_.size() + bools_.size() + doubles_.size();
  }

  const vector<double>& doubles() const { return doubles_; }
  const vector<int32>& ints() const { return ints_; }
  const vector<bool>& bools() const { return bools_; }

  void set_double(size_t i, double val) { doubles_[i] = val; }
  void set_int(size_t i, int32 val) { ints_[i] = val; }
  void set_bool(size_t i, bool val) { bools_[i] = val; }
  void set_value(const ControlValuesThreshold& thesh, size_t dim_id);
  void clear() { doubles_.clear(); ints_.clear(); bools_.clear(); }

  Eigen::VectorXd ToVector() const;

 private:
  vector<double> doubles_;
  vector<int32> ints_;
  vector<bool> bools_;

  friend class ControlValuesHelper;
  friend bool operator<(const ControlValues& v1, const ControlValues& v2);
  friend bool operator==(const ControlValues& v1, const ControlValues& v2);
  friend std::ostream& operator<<(std::ostream& os, const ControlValues& handle);
};

typedef ControlValues Sample;

// CONTROL VALUES THRESHOLD ----------------------------------------------------

// Encodes a threshold for an arbitrary control type (e.g. int, double)
class ControlValuesThreshold {
 public:
  ControlValuesThreshold(const int32 dim_id,
                         VarInfoSpec::VarType type)
      : dimension_id_(dim_id), type_(type),
        double_val_(0), int_val_(0), bool_val_(false) {}

  ControlValuesThreshold(const ThresholdSpec& spec);

  void ToSpec(ThresholdSpec* spec) const;

  const int32& dimension_id() const { return dimension_id_; }
  VarInfoSpec::VarType type() const { return type_; }
  double double_val() const { return double_val_; }
  int32 int_val() const { return int_val_; }
  bool bool_val() const { return bool_val_; }

  void set_double(double val) { double_val_ = val; }
  void set_int(int val) { int_val_ = val; }
  void set_bool(bool val) { bool_val_ = val; }

 private:
  const int32 dimension_id_;
  VarInfoSpec::VarType type_;
  double double_val_;
  int32 int_val_;
  bool bool_val_;
};

// OPERATORS -------------------------------------------------------------------
bool operator<(const ControlValues& v1, const ControlValues& v2);
bool operator==(const ControlValues& v1, const ControlValues& v2);

bool operator<(const ControlValues& v1, const ControlValuesThreshold& v2);

std::ostream& operator<<(std::ostream& os, const ControlValues& v);

// CONTROL VALUES HELPER -------------------------------------------------------

// Helper for encoding/decoding control values for any geometry template
// intantiation from e.g. string, key of the sample in the data structure, etc.
class ControlValuesHelper {
 public:
  ControlValuesHelper(const FabGeometryTemplate& tpl);

  ControlValuesHelper(const string& control_names,
                      const TemplateParams& params);

  void SetControls(const ControlValues& values,
                   FabGeometryTemplate* tpl) const;

  bool SetControlFromStream(const string& control_name,
                            std::istream& is,
                            ControlValues* values) const;

  void Initialize(ControlValues* values) const;
  void FillFromEncoding(const string& encoded, ControlValues* v) const;
  void FillFromTemplate(const FabGeometryTemplate* tpl, ControlValues* v) const;
  string GetKey(const ControlValues& v) const;
  string GetEncoding(const ControlValues& v) const;

  KeyUtil& key_util() { return key_helper_; }

  const KeyUtil& key_util() const { return key_helper_; }

  const vector<TemplateParams::ControlInfo>& controls() const {
    return key_helper_.controls();
  }

  int control_index(const string& name) {
    int i = 0;
    for (const TemplateParams::ControlInfo& cinfo : controls()) {
      if (cinfo.name() == name) {
        return i;
      }
      ++i;
    }
    LOG(ERROR) << "Control not found: \"" << name << "\"";
    return -1;
  }

 private:
  KeyUtil key_helper_;
};

// HYPERCUBOID -----------------------------------------------------------------

class PrecompLookup;

// Misnomer. Represents a single hyperrectangle in the desing space with the
// sample at its min and max vertex and an array of all samples in deterministic
// order. Does not actually store any precomputation results but just encodes
// the structure in the k-d tree.
class Hypercuboid {
 public:
  Hypercuboid(const Sample* min,
              const Sample* max,
              const vector<const Sample*>& samples);

  // Shallow copy
  Hypercuboid(const Hypercuboid& h);

  bool is_leaf() const;

  int n_leaves() const;

  // Returns leaves and their depths
  void GetLeaves(set<std::pair<const Hypercuboid*, int> >* leaves,
                 int start_depth = 0) const;
  void GetLeaves(vector<std::pair<Hypercuboid*, int> >* leaves,
                 int start_depth = 0);

  int tree_depth(int start_depth = 0) const;

  int n_dims() const { return min_->size(); }

  const Sample* smin() const { return min_; }

  const Sample* smax() const { return max_; }

  boost::shared_ptr<const ControlValuesThreshold> thresh() const {
    return thresh_;
  }

  Hypercuboid* left() const { return left_.get(); }
  Hypercuboid* right() const { return right_.get(); }

  // Finds the smallest child hypercuboid containing s;
  // else NULL if not contained.
  const Hypercuboid* Find(const Sample& s) const;

  const vector<const Sample*>& samples() const { return samples_; }

  bool Contains(const Sample& s) const;

  void GetVertexNeighbors(const int v_idx, vector<int>* neighbors) const;

  // Converts a single index to dimension-based indices, each of which
  // is either 0 (the min size) or 1 (the max size)
  void ToDimensionalIndices(const int idx, vector<int>* indices) const;

  static void ToDimensionalIndices(
      const int n_dims, const int idx, vector<int>* indices);

  // Converts dimensional-based indices to a single index that can be used
  // to access Hypercuboid's samples
  static int ToSingleIndex(const vector<int>& dimension_indices);

  //  int GetSubdivisionDimension(const PrecompLookup& lookup);

  void Split(const PrecompLookup& lookup,
             boost::shared_ptr<ControlValuesThreshold>& thresh,
             vector<const Sample*>* new_samples);

  void Split(boost::shared_ptr<ControlValuesThreshold>& thresh,
             Hypercuboid* left, Hypercuboid* right);

  // The range of each dimension is normalized to 1
  double NormalizedVolume(const PrecompLookup& lookup) const;

  // Returns true if sample lies on one of the hypercuboid edges
  bool FindContainingEdge(const Sample& s,
                          vector<const Sample*>* endpoints) const;

  bool FindValidityVertexSet(const Sample& s,
                             vector<const Sample*>* verts) const;

 private:
  Hypercuboid() {}
  const Sample* min_;
  const Sample* max_;

  boost::shared_ptr<ControlValuesThreshold> thresh_;
  boost::scoped_ptr<Hypercuboid> left_;  // smaller thresh value
  boost::scoped_ptr<Hypercuboid> right_;  // larger thresh value

  vector<const Sample*> samples_;
};

std::ostream& operator<<(std::ostream& os, const Hypercuboid& hc);

// INDEXER ---------------------------------------------------------------------

// Encodes control values for an arbitrary template with a small number of
// end-user-visible controls as an integer. Makes certain
// assumptions about the number of possible subdivisions per dimension.
class SampleIndexer {
 public:
  struct DimIndexInfo {
    DimIndexInfo(const Dimension& dim, int bins);

    double start;
    double bin_size;
  };

  SampleIndexer(const vector<const Dimension*>& dims);

  int64 GetIndex(const ControlValues& sample) const;

  int64 GetIndex(const vector<double>& sample_vals) const;

  int IndexInDim(double val, int dim_id) const;

 private:
  // 64 bits / 6 dims ==> 10 bits
  int bits_per_dim_;
  // 512
  int bins_per_dim_;

  vector<DimIndexInfo> infos_;
};

// DIMENSION -------------------------------------------------------------------

// Represents a single untyped dimension in the design space
class Dimension {
 public:
  Dimension(int id, const string& name = "")
      : id_(id), name_(name) {}

  const string& name() const { return name_; }

  virtual int n_cuts() const = 0;

  virtual string DebugCutString(int cut_id) const = 0;

  int dim_id() const { return id_; }

  virtual double mean() const = 0;

  virtual double range() const = 0;

  virtual double min_val() const = 0;

  virtual void LogInfo() const = 0;

 private:
  Dimension() {}

  int id_;
  string name_;
};

// Represents a typed dimension of type T in the design space
template <class T>
class DimensionTpl : public Dimension {
 public:
  DimensionTpl(int id, const string& name = "") : Dimension(id, name) {}

  void AddCut(const T& val);

  const vector<T>& cuts() const { return cuts_; }

  const T value(int cut_id) const { return cuts_[cut_id]; }

  virtual void LogInfo() const {
    std::stringstream ss;
    ss << "Dimension " << name() << " cuts: ";
    for (int c = 0; c < cuts().size(); ++c) {
      if (c > 0) {
        ss << ", ";
      }
      ss << cuts()[c];
    }
    LOG(INFO) << ss.str();
  }

  virtual string DebugCutString(int cut_id) const {
    std::stringstream ss;
    ss << value(cut_id);
    return ss.str();
  }

  void GetNeighboringCuts(const T& val,
                          int* prev_cut_id,
                          int* next_cut_id) const;

  int GetMatchingCut(const T& val) const;

  ControlValuesThreshold* SplitBetweenCuts(int min_cut, int next_cut);

  virtual int n_cuts() const { return cuts_.size(); }

  double mean() const {
    if (cuts_.size() < 1) return 0;
    if (cuts_.size() < 2) return cuts_[0];
    return static_cast<double>(cuts_.back() + cuts_[0]) / 2.0;
  }

  double range() const {
    if (cuts_.size() < 2) return 1.0;
    return fabs(static_cast<double>(cuts_.back() - cuts_[0]));
  }

  double min_val() const {
    if (cuts_.empty()) {
      return 0;
    }
    return cuts_[0];
  }

 private:
  mutable vector<T> cuts_;
};

template <>
ControlValuesThreshold* DimensionTpl<bool>::SplitBetweenCuts(int min_cut, int next_cut);

template <>
ControlValuesThreshold* DimensionTpl<int32>::SplitBetweenCuts(int min_cut, int next_cut);

template <>
ControlValuesThreshold* DimensionTpl<double>::SplitBetweenCuts(int min_cut, int next_cut);


// PROPERTY SUMMARY ------------------------------------------------------------

// An unsophisticated and not generalized summary of all properties / checks
// evaluated at a particular point in the design space.
class PropertySummary {
 public:
  PropertySummary() {}

  PropertySummary(const PropertyValuesSpec& props);

  PropertySummary(const PropertySummary& other);

  void Reset(const PropertyValuesSpec& props);

  bool ChecksPassed() const;

  // One of:
  // "unsatisfiable_constraints"
  // "mesh_generation_failed"
  // "non_watertight"
  // "bad_simulation"
  // "unstable"
  // "unknown_error"
  const string& ErrorCode() const;

  const PropertySummarySpec& summary_proto() const { return summary_; }

 private:
  void Init(const PropertyValuesSpec& props);

  PropertySummarySpec summary_;
};


// PRECOMP LOOKUP --------------------------------------------------------------

//!
//! Base class for all representations of design space precomputation.
class PrecompLookup {
 public:
  //! Must be not-null helper to ensure controls can be appropriately read;
  //! takes ownership of the helper.
  PrecompLookup(ControlValuesHelper* cv_helper);

  virtual ~PrecompLookup();

  // Validity --------------------------
  bool IsValid(const Sample& value) const;

  virtual PropertySummary GetProperties(const Sample& value) const;

  // throws if none
  bool IsValidExact(const Sample* value) const;

  // returns NULL if none
  virtual const PropertySummary* GetPropertiesExact(const Sample* value) const;

  // returns if has properties for a particular sample value
  bool HasProperties(const Sample* sample) const;

  // Runs *exact* validity checks on all the vertices
  bool AllVerticesValid(const vector<const Sample*>& vertices) const;

  bool OnValidityBorder(const Hypercuboid& cub) const;

  // Stats -----------------------------

  static void ComputeAndPrintStats(
      const PrecompLookup& lookup,
      const vector<const Hypercuboid*>& leaves);

  // Bounds on Validity ----------------
  //!
  //! Gets bounds for all dimensions surrounding the current value
  void GetBounds(const Sample& value,
                 vector<Bounds*>* bounds) const;

  //!
  //! Gets valid intervals for all dimensions with current value as
  //! the origin of the serach for valid intervals.
  void GetValidIntervals(const Sample& value,
                         vector<Bounds*>* interval_bounds) const;

  ValidityGrid2D* CreateValidityGrid(
      const string& control1, const string& control2,
      const Sample& current) const;

  // Distances -------------------------
  //! Finds the closest point to a given sample
  const Sample* ClosestPoint(const Sample& value,
                             double* delta_estimate) const;

  //! Finds distance between two existing samples.
  //! May be GeometryDelta OR some other metric based on e.g. validity;
  //! non-const because may require computation.
  double GetDistance(
      const Sample* s1,
      const Sample* s2,
      bool skip_delta_computation = false) const;

  // With an option to update
  double GetDistance(
      const Sample* s1,
      const Sample* s2,
      bool skip_delta_computation = false);

  // Editing ----------------------------

  //!
  //! Careful, if some past samples are overwritten, this
  //! will invalidate some dependent data sructures. Best to add all the samples
  //! at once.
  void AddSamples(const string& filemane);

  // Returns true if the sample is new
  virtual bool AddSample(const Sample* sample,
                         const PropertyValuesSpec* props = NULL);

  virtual void SetProperties(const Sample* sample,
                             const PropertyValuesSpec& props);

  virtual void DeleteProperties(const Sample* sample);

  // Info --------------------------------

  int n_dims() const;

  int n_cuts(int d) const;

  virtual int n_samples() const { return samples_.size(); }

  // Important: this is not hash, but number 0...(n_samples() - 1)
  virtual const Sample* getSampleAt(int sample_number) const {
    if (sample_number >= 0 && sample_number < samples_.size()) {
      return samples_[sample_number];
    }
    return NULL;
  }

  GeometryDeltaLookup* geo_delta_lookup() { return &geo_deltas_; }
  const GeometryDeltaLookup* geo_delta_lookup() const { return &geo_deltas_; }

  const Dimension& GetDimension(int dimension_idx) const;

  const ControlValuesHelper& cv_helper() const { return *cv_helper_; }
  ControlValues CurrentControlValues(const FabGeometryTemplate& tpl) const;
  void SetControls(const ControlValues& values, FabGeometryTemplate* tpl) const;

  // Finds samples using various keying versions
  const Sample* FindMatchingSample(const Sample& s) const;
  const Sample* FindMatchingSample(const string& key) const;
  virtual const Sample* FindMatchingSample(int64 key) const;

  void AddGeoDeltas(const GeoDeltasSpec& spec);
  void GeoDeltasToSpec(GeoDeltasSpec* spec) const;

  virtual void LogInfo() const {}

  virtual void GetAllSamples(vector<const Sample*>* samples) const {
    samples->insert(samples->begin(), samples_.begin(), samples_.end());
  }

  virtual void CopyAllLeaves(vector<const Hypercuboid*>* cuboids) const {}

 protected:

  int64 getSampleHash(const Sample& s) const {
    return getIndexer()->GetIndex(s);
  }

  // Pure virtual
  virtual Hypercuboid FindContainingHypercuboid(const Sample& s) const = 0;

  // Modifying dimensions
  void AddCuts(const Sample& s);

  // Indexing into dimensions
  int DimensionIdDouble(int double_dimension_idx) const;
  int DimensionIdInt(int int_dimension_idx) const;
  int DimensionIdBool(int bool_dimension_idx) const;
  vector<const Dimension*> dimensions() const {
    vector<const Dimension*> res;
    for (int i = 0; i < double_dims_.size(); ++i) {
      res.push_back(&double_dims_[i]);
    }
    for (int i = 0; i < int_dims_.size(); ++i) {
      res.push_back(&int_dims_[i]);
    }
    for (int i = 0; i < bool_dims_.size(); ++i) {
      res.push_back(&bool_dims_[i]);
    }
    return res;
  }

  // Indexing into dimension cuts
  int IndexOfLeftCut(const Sample& s, int dimension_id) const;
  int IndexOfMatchingCut(const Sample& s, int dimension_id) const;
  void GetCutIds(const Sample& s, vector<int>* ids) const;
  void SetValueToCutInDim(int dimension_id, int cut_id, Sample* s) const;

  // Bounds utils
  Bounds* CreateBounds(int dim_idx, int left_cut, int right_cut) const;

  // Creates bounds for a number of intervals
  Bounds* CreateBounds(int dimension_id,
                       const vector<int>& left_cuts,
                       const vector<int>& right_cut) const;

  virtual int GetSampleIdx(int64 key, const Sample* sample = NULL) const;

  // DATA ------------------------------
  // All the samples for which precomputation has been stored
  vector<const Sample*> samples_;

  // A map of all samples, returns index in samples_
  map<int64, size_t> sample_index_;

  // Map of all properties/validity computed for particular samples
  // (keyed by samples in samples_)
  map<const Sample*, PropertySummary> props_;

  // Cache of all change in geometry values
  GeometryDeltaLookup geo_deltas_;

  // Encodes sampled values for every dimension.
  vector<DimensionTpl<double> > double_dims_;
  vector<DimensionTpl<int32> > int_dims_;
  vector<DimensionTpl<bool> > bool_dims_;

  boost::scoped_ptr<ControlValuesHelper> cv_helper_;

 private:
  SampleIndexer* getIndexer() const {
    if (!indexer_) {
      indexer_ = new SampleIndexer(dimensions());
    }
    return indexer_;
  }

  mutable SampleIndexer* indexer_;
};

// GRID LOOKUP -----------------------------------------------------------------

class HybridLookup;

//!
//! Uniform grid lookup, used during initial bootstrapping stage.
class GridLookup : public PrecompLookup {
 public:
  //!
  //! For example:
  //! control_names "pt_x,pt_y,n_sides"
  //! cuts_in_dimension:  "2.00 3.00 4.05"
  //!                     "1.0 1.5 1.7"
  //!                     "3 4 5 6 7 8"
  //! samples_file: "/tmp/out.txt"
  //!               containing one encoded PropertyValuesSpec per line
  GridLookup(const string& control_names,
             const vector<string>& cuts_in_dimension,
             const string& samples_file,
             const FabGeometryTemplate* tpl);

  GridLookup(const GridLookupSpec& spec,
             const FabGeometryTemplate* tpl);

  virtual void CopyAllLeaves(vector<const Hypercuboid*>* cuboids) const;

  virtual void AllCuboids(vector<Hypercuboid*>* cuboids) const;

  virtual void LogInfo() const;

 protected:
  void Init(const string& control_names,
            const vector<string>& cuts_in_dimension,
            const string& samples_file,
            const FabGeometryTemplate* tpl);

  int ToSingleIndex(const vector<int>& indices) const;

  void ToDimensionalIndices(const int index, vector<int>* indices) const;

  string DebugIndicesString(const vector<int>& indices) const;

  void LoopOverD(const int d,
                 const vector<int>& smin_index,
                 vector<Hypercuboid*>* cuboids) const;

  virtual Hypercuboid FindContainingHypercuboid(const Sample& s) const;

  // III
  // virtual int GetSampleIdx(const string& key, const Sample* sample = NULL) const;
  virtual int GetSampleIdx(int64 key, const Sample* sample = NULL) const;

  // Store samples as an N-dimensional array in samples_,
  // where dimensions are ordered as:
  // d0... - double dimensions
  // dk... - int dimensions
  // ...dN - bool dimensions

  friend class HybridLookup;
};

// HYBRID LOOKUP ---------------------------------------------------------------

//!
//! Represents the final result of design space precomputation:
//! a uniform grid of samples in the design space, where every grid
//! hyperrectangle is a k-d tree of samples with a hyperrectangle at every leaf.
class HybridLookup : public PrecompLookup {
 public:
  HybridLookup(const HybridLookupSpec& spec,
               const FabGeometryTemplate* tpl);

  void AddCuboids(const HybridLookupSpec& spec);

  void CuboidsToSpec(HybridLookupSpec* spec) const;

  ~HybridLookup();

  Hypercuboid* root(int i) { return roots_lin_[i]; }

  int n_roots() const { return roots_lin_.size(); }

  int n_leaves() const;

  virtual int n_samples() const { return samples_.size() + grid_->n_samples(); }

  // Important: this is not hash, but number 0...(n_samples() - 1)
  virtual const Sample* getSampleAt(int sample_number) const {
    if (sample_number < grid_->n_samples()) {
      return grid_->getSampleAt(sample_number);
    }
    return PrecompLookup::getSampleAt(sample_number - grid_->n_samples());
  }

  virtual const PropertySummary* GetPropertiesExact(const Sample* value) const;

  bool SubdivideOne(Hypercuboid* leaf,
                    const int dimension,   // can be -1 for no subdivision
                    vector<const Sample*>* new_samples);

  void GetSamplesWithoutProperties(vector<const Sample*>* unsampled_pts) const;

  virtual const Sample* FindMatchingSample(int64 key) const;
  using PrecompLookup::FindMatchingSample;

  virtual void LogInfo() const;

  void GetLeafDepths(vector<int>* depths) const;

  virtual void GetAllSamples(vector<const Sample*>* samples) const {
    grid_->GetAllSamples(samples);
    PrecompLookup::GetAllSamples(samples);
  }

  void CopyAllLeaves(vector<const Hypercuboid*>* cuboids) const;

 protected:
  Hypercuboid* CuboidFromSpec(const HybridLookupSpec::HypercuboidSpec& spec);

  void CuboidToSpec(const Hypercuboid& cuboid,
                    HybridLookupSpec::HypercuboidSpec* spec) const;

  virtual Hypercuboid FindContainingHypercuboid(const Sample& s) const;

  vector<Hypercuboid*> roots_lin_;  // linearly indexed
  map<const Sample*, Hypercuboid*> roots_;  // allows cross-reference with grid
  boost::scoped_ptr<GridLookup> grid_;
};


// DEFINITIONS -----------------------------------------------------------------

template <class T>
void DimensionTpl<T>::AddCut(const T& val) {
  try {
    int exists = GetMatchingCut(val);
    VLOG(3) << "Cut " << val << " already exists.";
  } catch (std::runtime_error& e) {
    VLOG(2) << "Adding new cut " << val << " to dimension " << name();
    cuts_.push_back(val);
    std::sort(cuts_.begin(), cuts_.end());
  }
}

template <class T>
void DimensionTpl<T>::GetNeighboringCuts(const T& val,
                                         int* prev_cut_id,
                                         int* next_cut_id) const {
  CHECK_GT(cuts_.size(), 0);
  typename vector<T>::const_iterator it =
      std::lower_bound(cuts_.begin(), cuts_.end(), val);
  *next_cut_id = it - cuts_.begin();
  *prev_cut_id = *next_cut_id - 1;

  // If the value is on the cut, can go either way
  if (*prev_cut_id < 0 && KeyUtil::AreTwoEqual<T>(val, cuts_[0])) {
    *prev_cut_id = 0;
    *next_cut_id = 1;
  } else if (*next_cut_id >= cuts_.size() &&
             KeyUtil::AreTwoEqual<T>(val, cuts_.back())) {
    --(*prev_cut_id);
    --(*next_cut_id);
  }

  if (*prev_cut_id < 0 || *next_cut_id >= cuts_.size()) {
    LOG(ERROR) << "Unbounded value " << val << " for region: "
               << cuts_[0] << ", " << cuts_.back() << " in dim " << name();
    throw runtime_error("Unbounded value");
  }
}

template <class T>
int DimensionTpl<T>::GetMatchingCut(const T& val) const {
  for (int i = 0; i < cuts_.size(); ++i) {
    if (KeyUtil::AreTwoEqual<T>(val, cuts_[i])) {
      return i;
    }
  }

  stringstream ss;
  ss << "No exact match cut for value ";
  ss << val << " in dimension " << name();
  VLOG(2) << ss.str();
  throw runtime_error(ss.str());
}

}  // namespace mit_plato


#endif  // _FAB_GEOMETRY_TEMPLATE_TEMPLATE_PRECOMP_LOOKUP_H__
