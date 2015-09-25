#include <algorithm>
#include <fstream>
#include <iostream>
#include <math.h>
#include <sstream>
#include <stdexcept>
#include <stdlib.h>
using std::runtime_error;
using std::stringstream;
using std::cout;
using std::endl;

#include <iostream>     // std::cout, std::fixed
#include <iomanip>      // std::setprecision
using std::cout;
using std::endl;

#include <utility>      // std::rel_ops
using namespace std::rel_ops;

#include <boost/filesystem.hpp>
#include <boost/regex.hpp>
#include <Eigen/LU>
using namespace Eigen;

#include <GL/gl.h>
#include <GL/glut.h>

#include <boost/algorithm/string/replace.hpp>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <google/protobuf/text_format.h>

#include "fab/geometry/convert.h"
#include "fab/geometry/tri_mesh.h"
#include "fab/geometry/template/checks.h"
#include "fab/geometry/template/shape.h"
#include "fab/geometry/template/search_paths.h"
#include "fab/geometry/template/precomp/lookup.h"
#include "fab/geometry/template/template_params.h"
#include "fab/geometry/template/precomp/aws_util.h"

DEFINE_bool(use_geometry_deltas, false,
            "If true, requires geometrty deltas for the lookup to function.");
DEFINE_bool(ignore_missing_deltas, false,
            "If true, ignores (hopefully small) number of missing deltas");
DEFINE_bool(use_new_closest_pt, false,
            "If true uses simpler scaling to compute closest vertex.");

namespace mit_plato {

////////////////////////////////////////////////////////////////////////////////
// SOME CODE REMOVED
////////////////////////////////////////////////////////////////////////////////


// HYPERCUBOID -----------------------------------------------------------------

Hypercuboid::Hypercuboid(const Sample* min,
                         const Sample* max,
                         const vector<const Sample*>& samples)
    : min_(min),
      max_(max),
      samples_(samples),
      left_(NULL),
      right_(NULL){
  CHECK_EQ(min_->size(), max_->size());
  CHECK_EQ(samples_.size(), pow(2, min_->size()));
}

Hypercuboid::Hypercuboid(const Hypercuboid& h)
    : min_(h.min_),
      max_(h.max_),
      samples_(h.samples_),
      left_(NULL),
      right_(NULL){
  CHECK_EQ(min_->size(), max_->size());
  CHECK_EQ(samples_.size(), pow(2, min_->size()));
}

int Hypercuboid::n_leaves() const {
  if (is_leaf()) {
    return 1;
  } else {
    return left_->n_leaves() + right_->n_leaves();
  }
}

void Hypercuboid::GetLeaves(vector<std::pair<Hypercuboid*, int> >* leaves,
                            int start_depth) {
  if (is_leaf()) {
    leaves->push_back(std::make_pair(this, start_depth));
  } else {
    left_->GetLeaves(leaves, start_depth + 1);
    right_->GetLeaves(leaves, start_depth + 1);
  }
}

void Hypercuboid::GetLeaves(set<std::pair<const Hypercuboid*, int> >* leaves,
                            int start_depth) const {
  if (is_leaf()) {
    leaves->insert(std::make_pair(this, start_depth));
  } else {
    left_->GetLeaves(leaves, start_depth + 1);
    right_->GetLeaves(leaves, start_depth + 1);
  }
}

int Hypercuboid::tree_depth(int start_depth) const {
  if (is_leaf()) {
    return start_depth;
  } else {
    return fmax(left_->tree_depth(start_depth + 1),
                right_->tree_depth(start_depth + 1));
  }
}

bool Hypercuboid::is_leaf() const {
  if (left_ == NULL || right_ == NULL || thresh_ == NULL) {
    CHECK(left_ == NULL);
    CHECK(right_ == NULL);
    CHECK(thresh_ == NULL);
    return true;
  }
  return false;
}

const Hypercuboid* Hypercuboid::Find(const Sample& s) const {
  if (!Contains(s)) {
    VLOG(3) << "Hypercuboid does not contain sample: " << s;
    return NULL;
  }
  if (is_leaf()) return this;

  if (s < *thresh_) {
    return left_->Find(s);
  } else {
    return right_->Find(s);
  }
}

bool Hypercuboid::Contains(const Sample& s) const {
  // Is it approximately one of the endpoints?
  for (const Sample* sample : samples_) {
    if (*sample == s) {
      return true;
    }
  }

  // Is it bounded by the min/max
  return s >= *min_ && s <= *max_;
}

void Hypercuboid::ToDimensionalIndices(
    const int idx, vector<int>* indices) const {
  return ToDimensionalIndices(n_dims(), idx, indices);
}

void Hypercuboid::ToDimensionalIndices(
    const int n_dims, const int idx, vector<int>* indices) {
  int rem = idx;
  int divisor = pow(2, n_dims - 1);
  indices->resize(n_dims);
  for (int dim_id = n_dims - 1; dim_id >= 0; --dim_id) {
    div_t div_res = div(rem, divisor);
    (*indices)[dim_id] = div_res.quot;
    rem = div_res.rem;
    divisor /= 2;
  }
}

int Hypercuboid::ToSingleIndex(const vector<int>& dimension_indices) {
  int res = 0;
  int mult = 1;
  for (vector<int>::const_iterator it = dimension_indices.begin();
       it != dimension_indices.end(); ++it) {
    res += *it * mult;
    mult *= 2;
  }
  return res;
}


// E.g. if s is on the face of a hypercuboid, we look only at the face vertices
// to estimate if s is valid, but if s is inside a hypercuboid, we look at all
// hypercuboid vertices. This is a generalized version for highger D.
bool Hypercuboid::FindValidityVertexSet(const Sample& s,
                                        vector<const Sample*>* verts) const {
  if (!smin() || !smax()) {
    LOG(ERROR) << "Finding Validity Vertex Set: Hypercuboid has no min/max values!";
    return false;
  }
  VectorXd min_vec = smin()->ToVector();
  VectorXd max_vec = smax()->ToVector();
  VectorXd vec = s.ToVector();
  if (vec.rows() != n_dims()) {
    return false;
  }

  // Find if s shares any of the cuts with s or with min
  vector<bool> exact_cut_match(n_dims(), false);
  vector<double> exact_matches(n_dims(), 0);
  for (int d = 0; d < n_dims(); ++d) {
    if (KeyUtil::AreTwoEqual(vec[d], min_vec[d]) ||
        KeyUtil::AreTwoEqual(vec[d], max_vec[d])) {
      exact_cut_match[d] = true;
      exact_matches[d] = vec[d];
    }
  }

  for (const Sample* vert : samples_) {
    VectorXd vert_vec = vert->ToVector();
    bool to_include = true;
    for (int d = 0; d < n_dims(); ++d) {
      if (exact_cut_match[d] &&
          !KeyUtil::AreTwoEqual(vec[d], vert_vec[d])) {
        to_include = false;
        break;
      }
    }
    if (to_include) {
      verts->push_back(vert);
    }
  }
  if (verts->size() != samples_.size()) {
    VLOG(2) << "Using smaller validity set of " << verts->size()
            << " for sample " << s << " within " << *this;
  }
  return true;
}

bool Hypercuboid::FindContainingEdge(const Sample& s,
                                     vector<const Sample*>* endpoints) const {
  // Sample lies on edge if all of its values except one are
  // equal to min or max values

  if (!smin() || !smax()) {
    LOG(ERROR) << "Finding Edge: Hypercuboid has no min/max values!";
    return false;
  }
  VectorXd min_vec = smin()->ToVector();
  VectorXd max_vec = smax()->ToVector();
  VectorXd vec = s.ToVector();
  if (vec.rows() != n_dims()) {
    return false;
  }

  int non_equal_dims = 0;
  int variable_dim = 0;
  vector<int> dim_indices;
  for (int d = 0; d < n_dims(); ++d) {
    if (KeyUtil::AreTwoEqual(vec[d], min_vec[d])) {
      dim_indices.push_back(0);
    } else if (KeyUtil::AreTwoEqual(vec[d], max_vec[d])) {
      dim_indices.push_back(1);
    } else {
      ++non_equal_dims;
      if (non_equal_dims > 1) {
        return false;
      }
      dim_indices.push_back(0);
      variable_dim = d;
    }
  }

  dim_indices[variable_dim] = 0;
  endpoints->push_back(samples_[ToSingleIndex(dim_indices)]);
  dim_indices[variable_dim] = 1;
  endpoints->push_back(samples_[ToSingleIndex(dim_indices)]);
  return true;
}

void Hypercuboid::GetVertexNeighbors(
    const int v_idx, vector<int>* neighbors) const {
  // Convert a single index to dimension-based indices
  vector<int> dim_indices;
  ToDimensionalIndices(v_idx, &dim_indices);

  for (int d = 0; d < n_dims(); ++d) {
    vector<int> dim_indices_copy = dim_indices;
    if (dim_indices_copy[d] == 0) {
      dim_indices_copy[d] = 1;
    } else {
      dim_indices_copy[d] = 0;
    }
    neighbors->push_back(ToSingleIndex(dim_indices_copy));
  }
}

void Hypercuboid::Split(boost::shared_ptr<ControlValuesThreshold>& thresh,
                        Hypercuboid* left, Hypercuboid* right) {
  if (!is_leaf()) {
    LOG(FATAL) << "Attempting to split non-leaf node!";
    return;
  }

  thresh_ = thresh;
  left_.reset(left);
  right_.reset(right);
}

void Hypercuboid::Split(const PrecompLookup& lookup,
                        boost::shared_ptr<ControlValuesThreshold>& thresh,
                        vector<const Sample*>* new_samples) {
  if (!is_leaf()) {
    LOG(FATAL) << "Attempting to split non-leaf node!";
    return;
  }

  thresh_ = thresh;

  // Now need to create child hypercuboids and create samples
  vector<const Sample*> left_samples, right_samples;
  left_samples.resize(samples_.size(), NULL);
  right_samples.resize(samples_.size(), NULL);

  map<int, const Sample*> boundary_samples_map;
  const int d = thresh_->dimension_id();
  for (int idx = 0; idx < samples_.size(); ++idx) {
    vector<int> indices;
    Hypercuboid::ToDimensionalIndices(n_dims(), idx, &indices);

    // We only need to create half as many new samples on the boundary
    // between left and right cube
    if (boundary_samples_map.find(idx) == boundary_samples_map.end()) {
      // Find sample in lookup

      const Sample* new_sample = NULL;
      {
        Sample maybe_new_sample(*samples_[idx]);
        maybe_new_sample.set_value(*thresh, d);

        const Sample* existing_sample =
            lookup.FindMatchingSample(maybe_new_sample);
        if (existing_sample) {
          new_sample = existing_sample;
          if (!lookup.HasProperties(new_sample)) {
            new_samples->push_back(new_sample);
          }
        } else {
          new_sample = new Sample(maybe_new_sample);
          new_samples->push_back(new_sample);
        }
      }

      boundary_samples_map[idx] = new_sample;

      vector<int> new_sample_indices = indices;
      if (new_sample_indices[d] == 0) {
        new_sample_indices[d] = 1;
      } else {
        new_sample_indices[d] = 0;
      }
      boundary_samples_map[ToSingleIndex(new_sample_indices)] = new_sample;
    }

    if (indices[d] == 0) {
      left_samples[idx] = samples_[idx];
      right_samples[idx] = boundary_samples_map[idx];
    } else {
      right_samples[idx] = samples_[idx];
      left_samples[idx] = boundary_samples_map[idx];
    }
  }

  int smin_index = ToSingleIndex(vector<int>(n_dims(), 0));
  int smax_index = ToSingleIndex(vector<int>(n_dims(), 1));

  left_.reset(
      new Hypercuboid(
          left_samples[smin_index], left_samples[smax_index], left_samples));

  right_.reset(
      new Hypercuboid(
          right_samples[smin_index], right_samples[smax_index], right_samples));
}

double Hypercuboid::NormalizedVolume(const PrecompLookup& lookup) const {
  Eigen::VectorXd min_vec = min_->ToVector();
  Eigen::VectorXd max_vec = max_->ToVector();

  double volume = 1;
  for (int d = 0; d < n_dims(); ++d) {
    double side_length = (max_vec[d] - min_vec[d]) /
                         lookup.GetDimension(d).range();
    volume *= side_length;
  }
  return volume;
}

// DIMENSION -------------------------------------------------------------------

template <>
ControlValuesThreshold* DimensionTpl<bool>::SplitBetweenCuts(int min_cut, int next_cut) {
  LOG(INFO) << "Cannot split a boolean dimesion " << name();
  return NULL;
}

template <>
ControlValuesThreshold* DimensionTpl<int32>::SplitBetweenCuts(int min_cut, int next_cut) {
  CHECK_LT(min_cut, next_cut);

  ControlValuesThreshold* res = NULL;
  int new_val;
  if (min_cut + 1 < next_cut) {
    res = new ControlValuesThreshold(dim_id(), VarInfoSpec::TYPE_INT32);
    int cut_distance = (next_cut - min_cut) / 2;
    new_val = value(min_cut + cut_distance);
  } else {
    if (value(next_cut) - value(min_cut) < 2) {
      VLOG(1) << "Cannot further split an int dimension " << name() << " between "
              << value(min_cut) << " and " << value(next_cut);
      return NULL;
    }
    int half_distance = (value(next_cut) - value(min_cut)) / 2;

    new_val = value(min_cut) + half_distance;
    VLOG(2) << "Adding a cut " << new_val << " at position " << next_cut
            << " to dimension " << name();
    cuts_.insert(cuts_.begin() + next_cut,  // inserted before this position
                 new_val);
    res = new ControlValuesThreshold(dim_id(), VarInfoSpec::TYPE_INT32);
  }

  res->set_int(new_val);
  return res;
}

template <>
ControlValuesThreshold* DimensionTpl<double>::SplitBetweenCuts(int min_cut, int next_cut) {
  CHECK_LT(min_cut, next_cut);

  ControlValuesThreshold* res = NULL;
  double new_val;
  if (min_cut + 1 < next_cut) {
    res = new ControlValuesThreshold(dim_id(), VarInfoSpec::TYPE_DOUBLE);
    int cut_distance = (next_cut - min_cut) / 2;
    new_val = value(min_cut + cut_distance);
  } else {
    double half_distance = (value(next_cut) - value(min_cut)) / 2.0;
    if (half_distance < 0.001) {
      VLOG(1) << "Cannot further split an double dimension " << name() << " between "
              << value(min_cut) << " and " << value(next_cut);
      return NULL;
    }

    new_val = value(min_cut) + half_distance;
    VLOG(2) << "Adding a cut " << new_val << " at position " << next_cut
            << " to dimension " << name();
    cuts_.insert(cuts_.begin() + next_cut,  // inserted before this position
                 new_val);
    res = new ControlValuesThreshold(dim_id(), VarInfoSpec::TYPE_DOUBLE);
  }

  res->set_double(new_val);
  return res;
}

// PRECOMP LOOKUP --------------------------------------------------------------

PrecompLookup::PrecompLookup(ControlValuesHelper* cv_helper)
    : cv_helper_(CHECK_NOTNULL(cv_helper)),
      indexer_(NULL) {
  VLOG(2) << "Creating Precomp lookup";
}

PrecompLookup::~PrecompLookup() {
  for (const auto& p : props_) {
    //    delete p.second;
  }
  for (const auto& p : samples_) {
    delete p;
  }
  delete indexer_;
}

void PrecompLookup::AddSamples(const string& samples_file) {
  if (samples_file.empty()) {
    LOG(WARNING) << "No samples";
    return;
  }

  LOG(INFO) << "Reading samples from " << samples_file;

  // Read samples
  std::ifstream ifs(SearchPaths::AbsolutizePrecompPath(samples_file).c_str());
  for(string line; std::getline(ifs, line);) {
    if (line.empty()) continue;

    PropertyValuesSpec spec;
    if (!google::protobuf::TextFormat::ParseFromString(line, &spec)) {
      LOG(ERROR) << "Failed to parse line: " << line;
      continue;
    }
    Sample* sample = new Sample(*cv_helper_, spec.key());
    AddSample(sample, &spec);
    VLOG(3) << "Read Sample " << *sample << " from key " << spec.key();
  }

  LOG(INFO) << "Read " << samples_.size() << " samples from " << samples_file;
}

bool PrecompLookup::AddSample(const Sample* sample,
                              const PropertyValuesSpec* props) {
  VLOG(3) << "Adding sample " << *sample;
  bool is_new = true;
  int64 sample_key = getSampleHash(*sample);
  int idx = GetSampleIdx(sample_key, sample);
  VLOG(4) << "Got index: " << idx;
  if (idx < 0) {  // Easy! Just add new
    idx = samples_.size();
    samples_.push_back(NULL);
  } else {
    CHECK_LT(idx, samples_.size());
    if (samples_[idx] && samples_[idx] != sample) {
      VLOG(3) << "Sample already exists; replacing";
      is_new = false;
      DeleteProperties(samples_[idx]);
      delete samples_[idx];
    }
  }

  sample_index_[sample_key] = idx;
  samples_[idx] = sample;
  if (props) {
    SetProperties(sample, *props);
  }
  if (is_new) {
    AddCuts(*sample);
  }
  VLOG(2) << "Added sample " << *sample << " as "
          << (is_new ? "new" : "replaced") << " with key " << sample_key;
  return is_new;
}

void PrecompLookup::SetProperties(const Sample* sample,
                                  const PropertyValuesSpec& props) {
  // Check that have this sample
  vector<const Sample*>::const_iterator it =
      std::find(samples_.begin(), samples_.end(), sample);
  CHECK(it != samples_.end())
      << "Attempting to set properties for non-existent sample pointer "
      << *sample;

  if (!props.not_yet_processed()) {
    props_[sample].Reset(props);
  }
}

bool PrecompLookup::HasProperties(const Sample* sample) const {
  try {
    bool props = IsValidExact(sample);
    return true;
  } catch (runtime_error& e) {
    return false;
  }
}

void PrecompLookup::DeleteProperties(const Sample* sample) {
  props_.erase(sample);
}

ValidityGrid2D* PrecompLookup::CreateValidityGrid(
    const string& control1, const string& control2,
    const Sample& current) const {
  int dim_idx1 = cv_helper_->control_index(control1);
  int dim_idx2 = cv_helper_->control_index(control2);
  if (dim_idx1 < 0 || dim_idx2 < 0) {
    return NULL;
  }

  if (dim_idx1 >= double_dims_.size() ||
      dim_idx1 >= double_dims_.size()) {
    LOG(ERROR) << "Valid grid only implemented for dimensions of type double";
    return NULL;
  }

  // HACK! Only works for doubles
  const vector<double>& cuts1 =
      dynamic_cast<const DimensionTpl<double>&>(GetDimension(dim_idx1)).cuts();
  const vector<double>& cuts2 =
      dynamic_cast<const DimensionTpl<double>&>(GetDimension(dim_idx2)).cuts();

  ValidityGrid2D* grid = new ValidityGrid2D(cuts1.size(), cuts2.size());
  for (int i = 0; i < cuts1.size(); ++i) {
    for (int j = 0; j < cuts2.size(); ++j) {
      Sample s = current;

      {
        std::stringstream ss;
        ss << cuts1[i];
        if (!cv_helper_->SetControlFromStream(control1, ss, &s)) {
          delete grid;
          return NULL;
        }
      }

      {
        std::stringstream ss;
        ss << cuts2[j];
        LOG(INFO) << "Setting control from stream \"" << ss.str() << "\" for val " << cuts2[j];
        if (!cv_helper_->SetControlFromStream(control2, ss, &s)) {
          delete grid;
          return NULL;
        }
      }

      bool valid = IsValid(s);
      grid->SetPoint(i, j, cuts1[i], cuts2[j], valid ? 0 : 1);
    }
  }
  return grid;
}

void PrecompLookup::ComputeAndPrintStats(
    const PrecompLookup& lookup,
    const vector<const Hypercuboid*>& leaves) {
  double total_volume(0), strict_valid_volume(0), vertex_invalid_volume(0);
  map<string, double> error_breakdown;
  for (const Hypercuboid* hc : leaves) {
    const double hc_volume = hc->NormalizedVolume(lookup);
    const double vertex_volume =
        hc_volume / static_cast<double>(hc->samples().size());
    total_volume += hc_volume;

    if (lookup.AllVerticesValid(hc->samples())) {
      strict_valid_volume += hc_volume;
    } else {
      for (const Sample* s : hc->samples()) {
        const PropertySummary* props = lookup.GetPropertiesExact(s);
        if (props) {
          if (!props->ChecksPassed()) {
            vertex_invalid_volume += vertex_volume;
            error_breakdown[props->ErrorCode()] += vertex_volume;
          }
        } else {
          LOG(ERROR) << "No properties for sample: " << *s;
          error_breakdown["No properties (weird)"] += 1;
        }
      }
    }
  }

  cout << "-------------------------------------------" << endl;
  cout << "Total volume: " << total_volume << endl;
  cout << "Strict valid volume: " << strict_valid_volume << endl;
  cout << "Vertex based valid volume: " << (1.0 - vertex_invalid_volume) << endl;

  cout << "Error breakdown by volume -----------------" << endl;
  for (const auto& pr : error_breakdown) {
    cout << pr.first << ": " << (pr.second / vertex_invalid_volume * 100)
              << " % of errors" << endl;
  }
  cout << "-------------------------------------------" << endl;
}

int PrecompLookup::n_dims() const {
  return double_dims_.size() + int_dims_.size() + bool_dims_.size();
}

ControlValues PrecompLookup::CurrentControlValues(
    const FabGeometryTemplate& tpl) const {
  return ControlValues(*cv_helper_, &tpl);
}

void PrecompLookup::SetControls(const ControlValues& values,
                                FabGeometryTemplate* tpl) const {
  cv_helper_->SetControls(values, tpl);
}

double PrecompLookup::GetDistance(const Sample* s1, const Sample* s2,
                                  bool skip_delta_computation) const {
  VLOG(5) << "Getting distance between " << *s1 << " and " << *s2;
  bool valid1 = false;
  try {
    valid1 = IsValidExact(s1);
  } catch (std::runtime_error& e) {
    LOG(FATAL) << "CANNOT COMPUTE DISTANCE!!! - " << e.what();
  }
  bool valid2 = false;
  try {
    valid2 = IsValidExact(s2);
  } catch (std::runtime_error& e) {
    LOG(FATAL) << "CANNOT COMPUTE DISTANCE!!! - " << e.what();
  }
  VLOG(3) << "Getting distance between "
          << *s1 << " (" << (valid1 ? "valid" : "invalid") << ")"
          << " and "
          << *s2 << " (" << (valid2 ? "valid" : "invalid") << ")";

  if (!valid1 && !valid2) {
    return 0;  // minimal distance
  }
  if (valid1 != valid2) {
    return 1;  // maximal distance
  }
  if (!FLAGS_use_geometry_deltas || skip_delta_computation) {
    VLOG(1) << "SKIPPING DELTA COMPUTATION";
    return 0.5;
  }
  if (geo_deltas_.HasDelta(s1, s2)) {
    VLOG(1) << "Delta is cached --> RETREIVING";
    return geo_deltas_.GetGeometryDelta(s1, s2);
  } else {
    VLOG(1) << "Delta is not found --> ERROR!";
    if (FLAGS_ignore_missing_deltas) {
      return 0.1; // HACK
    }
  }
  throw runtime_error("Delta not stored.");
}

double PrecompLookup::GetDistance(const Sample* s1, const Sample* s2,
                                  bool skip_delta_computation) {
  double res = 0;
  try {
    res = const_cast<const PrecompLookup*>(this)->GetDistance(
        s1, s2, skip_delta_computation);
  } catch (runtime_error& e) {
    VLOG(1) << "Delta not stored --> COMPUTING";
    geo_deltas_.CacheGeometryDelta(*cv_helper_, s1, s2);
    res = geo_deltas_.GetGeometryDelta(s1, s2);
  }
  VLOG(1) << "Delta between " << *s1 << " and " << *s2 << " is " << res;
  return res;
}

void PrecompLookup::GeoDeltasToSpec(GeoDeltasSpec* spec) const {
  geo_deltas_.ToSpec(*cv_helper_, spec);
}

void PrecompLookup::AddGeoDeltas(const GeoDeltasSpec& spec) {
  for (const auto& dspec : spec.delta()) {
    const Sample* s1 = FindMatchingSample(
        Sample(*cv_helper_, dspec.sample_id1()));
    const Sample* s2 = FindMatchingSample(
        Sample(*cv_helper_, dspec.sample_id2()));
    if (!s1) {
      LOG(ERROR) << "Failed to find matching sample: " << dspec.sample_id1();
      continue;
    }
    if (!s2) {
      LOG(ERROR) << "Failed to find matching sample: " << dspec.sample_id2();
      continue;
    }
    geo_deltas_.SetDelta(s1, s2, dspec.value());
  }
}

const Sample* PrecompLookup::ClosestPoint(const Sample& value,
                                          double* delta_estimate) const {
  const Sample* res = FindMatchingSample(value);
  if (res) {
    if (IsValidExact(res)) {
      LOG(INFO) << "Closest pt: Found matching VALID sample "
                << *res << " for value "<< value;
      return res;
    } else {
      LOG(INFO) << "Closest pt: Found matching IN-VALID sample "
                << *res << " for value "<< value;
      res = NULL;
    }
  }

  Hypercuboid cub = FindContainingHypercuboid(value);
  LOG(INFO) << "Closest pt: Found containing hypercuboid "
            << cub << " for value " << value;
  // Find average scaling in any one given direction
  vector<double> ave_lengths;
  for (int d = 0; d < n_dims(); ++d) {
    double sum(0);
    int num(0);
    for (int idx = 0; idx < cub.samples().size(); ++idx) {
      // TODO: the work is done twice, b/c delta is direction-independent
      const Sample* s1 = cub.samples()[idx];
      vector<int> indices;
      cub.ToDimensionalIndices(n_dims(), idx, &indices);
      CHECK_EQ(indices.size(), n_dims());

      indices[d] = (indices[d] == 0 ? 1 : 0);
      CHECK_LT(cub.ToSingleIndex(indices), cub.samples().size());
      const Sample* s2 = cub.samples()[cub.ToSingleIndex(indices)];
      CHECK(s1);
      CHECK(s2);

      double dist = GetDistance(s1, s2);
      if (mds::ApproxEqual(dist, 1.0) || mds::ApproxEqual(dist, 0.0)) {
        continue;
      }
      VLOG(1) << "Using dist " << dist;
      sum += dist;
      num++;
    }
    if (num == 0) {
      ave_lengths.push_back(1.0);
    } else {
      ave_lengths.push_back(sum/num);
    }
    LOG(INFO) << "Dimension " << d << " scaled by " << ave_lengths.back();
  }

  vector<double> distances;
  VectorXd query = value.ToVector();
  for (int i = 0; i < cub.samples().size(); ++i) {
    if (*(cub.samples()[i]) == value) {
      return cub.samples()[i];
    }
    if (!IsValidExact(cub.samples()[i])) {
      distances.push_back(1000000);
    } else {
      // Compute scaled vertex distance
      VectorXd vertex = cub.samples()[i]->ToVector();
      VectorXd diff = vertex - query;
      VLOG(1) << "Diff vector: " << diff;
      for (int d = 0; d < n_dims(); ++d) {
        diff[d] = diff[d] * ave_lengths[d];
      }
      VLOG(1) << "Scaled diff vector: " << diff;

      distances.push_back(diff.norm());
      VLOG(1) << "Computed distance from " << value << " to "
              << cub.samples()[i] << " = " << distances.back();
    }
  }

  const Sample* res = cub.samples()[0];
  *delta_estimate = distances[0];
  for (int i = 1; i < distances.size(); ++i) {
    if (distances[i] < *delta_estimate) {
      *delta_estimate = distances[i];
      res = cub.samples()[i];
    }
  }
  return res;
}

PropertySummary PrecompLookup::GetProperties(const Sample& value) const {
  PropertySummary props;
  double delta = 0;
  const Sample* closest = ClosestPoint(value, &delta);
  if (closest) {
    const PropertySummary* summary = GetPropertiesExact(closest);
    if (summary) {
      props = *summary;
    }
  }
  return props;
}

const PropertySummary* PrecompLookup::GetPropertiesExact(
    const Sample* s) const {
  const auto& it = props_.find(s);
  if (it == props_.end()) {
    stringstream ss;
    ss << "No props for sample " << *s;
    VLOG(4) << ss.str();
    return NULL;
  } else {
    VLOG(4) << "Got props for sample " << *s;
  }
  return &(it->second);
}

bool PrecompLookup::IsValidExact(const Sample* s) const {
  const PropertySummary* props = GetPropertiesExact(s);
  if (!props) {
    stringstream ss;
    ss << "IsValidExact: No props for sample " << *s;
    throw runtime_error(ss.str());
  }
  return props->ChecksPassed();
}

bool PrecompLookup::OnValidityBorder(const Hypercuboid& cub) const {
  int valid(0), invalid(0);
  for (const Sample* s : cub.samples()) {
    if (IsValidExact(s)) {
      ++valid;
    } else {
      ++invalid;
    }
  }
  return valid > 0 && invalid > 0;
}

bool PrecompLookup::AllVerticesValid(const vector<const Sample*>& samples) const {
  for (const Sample* s : samples) {
    if (!IsValidExact(s)) return false;
  }
  return true;
}

bool PrecompLookup::IsValid(const Sample& value) const {
  // TODO: properly handle grid/hybrid interaction here
  try {
    return IsValidExact(&value);
  } catch (runtime_error& e) {
    VLOG(3) << "Validity: no pointer match to " << value;
    try {
      const Sample* s = FindMatchingSample(value);
      if (!s) {
        VLOG(3) << "Validity: no matching sample to " << value;
        throw runtime_error("sample not found");
      } else {
        return IsValidExact(s);
      }
    } catch (runtime_error& e) {
      VLOG(3) << "Validity: looking at hypercuboid of " << value;
      Hypercuboid cub = FindContainingHypercuboid(value);
      vector<const Sample*> endpoints;
      if (cub.FindValidityVertexSet(value, &endpoints)) {
        return AllVerticesValid(endpoints);
      } else {
        return AllVerticesValid(cub.samples());
      }
    }
  }
}

Bounds* PrecompLookup::CreateBounds(int dimension_id,
                                    const vector<int>& left_cuts,
                                    const vector<int>& right_cuts) const {
  CHECK_EQ(left_cuts.size(), right_cuts.size());
  CHECK_GT(left_cuts.size(), 0);

  if (dimension_id < double_dims_.size()) {
    BoundsSpec::DoubleSpec spec;
    for (int i = 0; i < left_cuts.size(); ++i) {
      spec.add_minimum(
          double_dims_[dimension_id].value(left_cuts[i]));
      spec.add_maximum(
          double_dims_[dimension_id].value(right_cuts[i]));
    }
    return new DoubleBounds(spec);
  }
  dimension_id -= double_dims_.size();

  if (dimension_id < int_dims_.size()) {
    BoundsSpec::IntSpec spec;
    for (int i = 0; i < left_cuts.size(); ++i) {
      spec.add_minimum(
          int_dims_[dimension_id].value(left_cuts[i]));
      spec.add_maximum(
          int_dims_[dimension_id].value(right_cuts[i]));
    }
    return new IntBounds(spec);
  }
  dimension_id -= int_dims_.size();

  CHECK_EQ(left_cuts.size(), 1)
      << "Cannot have more than one valid interval for a bool value";

  if (left_cuts[0] == right_cuts[0]) {
    return new BoolBounds(bool_dims_[dimension_id].value(left_cuts[0]));
  } else {
    return new TrivialBounds();
  }
}

Bounds* PrecompLookup::CreateBounds(
    int dimension_id, int left_cut, int right_cut) const {
  vector<int> left_cuts;
  left_cuts.push_back(left_cut);

  vector<int> right_cuts;
  right_cuts.push_back(right_cut);

  return CreateBounds(dimension_id, left_cuts, right_cuts);
}

void PrecompLookup::SetValueToCutInDim(int dimension_id,
                                       int cut_id,
                                       Sample* s) const {
  if (dimension_id < double_dims_.size()) {
    s->set_double(dimension_id, double_dims_[dimension_id].value(cut_id));
    return;
  }
  dimension_id -= double_dims_.size();

  if (dimension_id < int_dims_.size()) {
    s->set_int(dimension_id, int_dims_[dimension_id].value(cut_id));
    return;
  }
  dimension_id -= int_dims_.size();

  s->set_bool(dimension_id, bool_dims_[dimension_id].value(cut_id));
}

const Dimension& PrecompLookup::GetDimension(int dimension_id) const {
  if (dimension_id < double_dims_.size()) {
    return double_dims_[dimension_id];
  }
  dimension_id -= double_dims_.size();

  if (dimension_id < int_dims_.size()) {
    return int_dims_[dimension_id];
  }
  dimension_id -= int_dims_.size();

  return bool_dims_[dimension_id];
}

int PrecompLookup::n_cuts(int dimension_id) const {
  return GetDimension(dimension_id).n_cuts();
}


void PrecompLookup::GetValidIntervals(
    const Sample& value, vector<Bounds*>* interval_bounds) const {
  VLOG(2) << "Getting valid intervals for sample" << value;
  for (int d = 0; d < n_dims(); ++d) {
    bool inside_interval = false;
    vector<int> endcuts;
    for (int cut = 0; cut < n_cuts(d); ++cut) {
      Sample s(value);
      SetValueToCutInDim(d, cut, &s);
      VLOG(4) << "Processing Sample (validity): " << s;

      if (IsValid(s)) {
        if (!inside_interval) {
          inside_interval = true;
          endcuts.push_back(cut);
        }
      } else {
        if (inside_interval) {
          inside_interval = false;
          endcuts.push_back(cut - 1);
        }
      }
    }
    if (inside_interval) {
      endcuts.push_back(n_cuts(d) - 1);
    }
    CHECK_EQ(0, endcuts.size() % 2)
        << "Odd number of interval endpoints: " << endcuts.size();
    if (endcuts.empty()) {
      LOG(ERROR) << "No valid intervals for sample " << value
                 << " and dimension " << d
                 << " ("<< GetDimension(d).name() << ")";
      endcuts.push_back(0);
      endcuts.push_back(0);
    }
    vector<int> left_cuts;
    vector<int> right_cuts;
    for (int i = 0; i < static_cast<int>(endcuts.size()) - 1; i+=2) {
      VLOG(6) << "E " << i << " endcuts: " << static_cast<int>(endcuts.size()) - 1;
      VLOG(6) << endcuts[i] << ", " << endcuts[i + 1];
      left_cuts.push_back(endcuts[i]);
      right_cuts.push_back(endcuts[i + 1]);
      VLOG(6) << "E -ok";
    }
    for (int i = 0; i < left_cuts.size(); ++i) {
      VLOG(6) << "Left: " << left_cuts[i];
      VLOG(6) << "Right: " << right_cuts[i];
    }

    VLOG(6) << "F";
    interval_bounds->push_back(CreateBounds(d, left_cuts, right_cuts));
    VLOG(6) << "G";
  }
}

void PrecompLookup::GetBounds(const Sample& value,
                              vector<Bounds*>* bounds) const {
  for (int d = 0; d < n_dims(); ++d) {
    int left_cut = IndexOfLeftCut(value, d);
    int right_cut = left_cut + 1;

    for (int cut = left_cut; cut >= 0; --cut) {
      Sample s(value);
      SetValueToCutInDim(d, cut, &s);
      if (!IsValid(s)) {
        left_cut = cut + 1;
        break;
      }
    }

    for (int cut = right_cut; cut < n_cuts(d); ++cut) {
      Sample s(value);
      SetValueToCutInDim(d, cut, &s);
      if (!IsValid(s)) {
        right_cut = cut - 1;
        break;
      }
    }

    bounds->push_back(CreateBounds(d, left_cut, right_cut));
  }
}

int PrecompLookup::DimensionIdDouble(int double_dimension_idx) const {
  return double_dimension_idx;
}

int PrecompLookup::DimensionIdInt(int int_dimension_idx) const {
  return double_dims_.size() + int_dimension_idx;
}

int PrecompLookup::DimensionIdBool(int bool_dimension_idx) const {
  return double_dims_.size() + int_dims_.size() + bool_dimension_idx;
}

int PrecompLookup::IndexOfLeftCut(const Sample& s, int dimension_id) const {
  const TemplateParams::ControlInfo& cinfo =
      cv_helper_->controls()[dimension_id];
  VLOG(5) << "s: " << s << " dim: " << dimension_id << " dims: " << double_dims_.size();

  int cut_id1(0), cut_id2(0);
  if (cinfo.info()->type() == VarInfoSpec::TYPE_DOUBLE) {
    double_dims_[dimension_id].GetNeighboringCuts(
        s.doubles()[dimension_id], &cut_id1, &cut_id2);
    return cut_id1;
  }
  dimension_id -= double_dims_.size();

  if (cinfo.info()->type() == VarInfoSpec::TYPE_INT32) {
    int_dims_[dimension_id].GetNeighboringCuts(
        s.ints()[dimension_id], &cut_id1, &cut_id2);
    return cut_id1;
  }
  dimension_id -= int_dims_.size();

  if (cinfo.info()->type() == VarInfoSpec::TYPE_BOOL) {
    bool_dims_[dimension_id].GetNeighboringCuts(
        s.bools()[dimension_id], &cut_id1, &cut_id2);
    return cut_id1;
  }
  LOG(FATAL) << "Not implemented for control type "
             << VarInfoSpec::VarType_Name(cinfo.info()->type());
  return cut_id1;
}

int PrecompLookup::IndexOfMatchingCut(const Sample& s, int dimension_id) const {
  const TemplateParams::ControlInfo& cinfo =
      cv_helper_->controls()[dimension_id];

  if (cinfo.info()->type() == VarInfoSpec::TYPE_DOUBLE) {
    return double_dims_[dimension_id].GetMatchingCut(s.doubles()[dimension_id]);
  }
  dimension_id -= double_dims_.size();

  if (cinfo.info()->type() == VarInfoSpec::TYPE_INT32) {
    return int_dims_[dimension_id].GetMatchingCut(s.ints()[dimension_id]);
  }
  dimension_id -= int_dims_.size();

  if (cinfo.info()->type() == VarInfoSpec::TYPE_BOOL) {
    return bool_dims_[dimension_id].GetMatchingCut(s.bools()[dimension_id]);
  }
  LOG(FATAL) << "Not implemented for control type "
             << VarInfoSpec::VarType_Name(cinfo.info()->type());
  return -1;
}

void PrecompLookup::GetCutIds(const Sample& s, vector<int>* ids) const {
  for (int d = 0; d < n_dims(); ++d) {
    ids->push_back(IndexOfMatchingCut(s, d));
  }
}

const Sample* PrecompLookup::FindMatchingSample(const Sample& s) const {
  // III // const string& key = cv_helper_->GetKey(s);
  int64 key = getSampleHash(s);
  return FindMatchingSample(key);
}

const Sample* PrecompLookup::FindMatchingSample(int64 key) const {
  map<int64, size_t>::const_iterator it = sample_index_.find(key);
  if (it != sample_index_.end()) {
    return samples_[it->second];
  }
  return NULL;
}

int PrecompLookup::GetSampleIdx(int64 key, const Sample* sample) const {
  map<int64, size_t>::const_iterator it = sample_index_.find(key);
  if (it != sample_index_.end()) {
    return it->second;
  }
  return -1;
}

const Sample* PrecompLookup::FindMatchingSample(const string& key) const {
  return FindMatchingSample(Sample(*cv_helper_, key));
}

void PrecompLookup::AddCuts(const Sample& s) {
  for (int i = 0; i < s.doubles().size(); ++i) {
    double_dims_[i].AddCut(s.doubles()[i]);
  }
  for (int i = 0; i < s.ints().size(); ++i) {
    int_dims_[i].AddCut(s.ints()[i]);
  }
  for (int i = 0; i < s.bools().size(); ++i) {
    bool_dims_[i].AddCut(s.bools()[i]);
  }
}

// GRID LOOKUP -----------------------------------------------------------------

GridLookup::GridLookup(const string& control_names,
                       const vector<string>& cuts_in_dimension,
                       const string& samples_file,
                       const FabGeometryTemplate* tpl)
    : PrecompLookup(new ControlValuesHelper(
        control_names, tpl->params_container())) {
  VLOG(2) << "Creating Grid lookup";
  Init(control_names, cuts_in_dimension, samples_file, tpl);
}

GridLookup::GridLookup(const GridLookupSpec& spec,
                       const FabGeometryTemplate* tpl)
    : PrecompLookup(new ControlValuesHelper(
        spec.control_names(), tpl->params_container())) {
  vector<string> cuts;
  for (const string& c : spec.cuts()) {
    cuts.push_back(c);
  }
  Init(spec.control_names(), cuts, spec.samples_file(), tpl);
}

void GridLookup::Init(const string& control_names,
                      const vector<string>& cuts_in_dimension,
                      const string& samples_file,
                      const FabGeometryTemplate* tpl) {
  // Fill in dimensions
  CHECK_EQ(cv_helper_->controls().size(), cuts_in_dimension.size());

  VLOG(1) << "Initializing grid";
  int grid_size = 1;
  for (int i = 0; i < cv_helper_->controls().size(); ++i) {
    const TemplateParams::ControlInfo& cinfo = cv_helper_->controls()[i];
    const string& cuts = cuts_in_dimension[i];

    if (cinfo.info()->type() == VarInfoSpec::TYPE_DOUBLE) {
      CHECK(int_dims_.empty() && bool_dims_.empty())
          << "Controls must be sorted as doubles, ints, bools";
      double_dims_.push_back(DimensionTpl<double>(double_dims_.size(), cinfo.name()));
      stringstream ss(cuts);
      while (ss.rdbuf()->in_avail() != 0) {
        double_dims_.back().AddCut(KeyUtil::ReadOrThrow<double>(ss));
      }
      grid_size *= double_dims_.back().n_cuts();
      CHECK_GT(double_dims_.back().n_cuts(), 1)
          << "Dimension " << cinfo.name() << " has too few cuts";
    } else if (cinfo.info()->type() == VarInfoSpec::TYPE_INT32) {
       CHECK(bool_dims_.empty())
           << "Controls must be sorted as doubles, ints, bools";
       int_dims_.push_back(DimensionTpl<int32>(
           double_dims_.size() + int_dims_.size(),
           cinfo.name()));
       stringstream ss(cuts);
       while (ss.rdbuf()->in_avail() != 0) {
         int_dims_.back().AddCut(KeyUtil::ReadOrThrow<int>(ss));
       }
       grid_size *= int_dims_.back().n_cuts();
       CHECK_GT(int_dims_.back().n_cuts(), 1)
           << "Dimension " << cinfo.name() << " has too few cuts";
    } else if (cinfo.info()->type() == VarInfoSpec::TYPE_BOOL) {
      bool_dims_.push_back(DimensionTpl<bool>(
          double_dims_.size() + int_dims_.size() + bool_dims_.size(),
          cinfo.name()));
      stringstream ss(cuts);
      while (ss.rdbuf()->in_avail() != 0) {
        bool_dims_.back().AddCut(KeyUtil::ReadOrThrow<bool>(ss));
      }
      grid_size *= bool_dims_.back().n_cuts();
      CHECK_GT(bool_dims_.back().n_cuts(), 1)
          << "Dimension " << cinfo.name() << " has too few cuts";
    }
  }
  VLOG(1) << "Read all the grid cuts";

  // Fill in the samples
  samples_.resize(grid_size, NULL);
  AddSamples(samples_file);

  // Make sure we have no NULLs
  int null_samples = 0;
  for (int s = 0; s < samples_.size(); ++s) {
    if (!samples_[s]) {
      ++null_samples;
      vector<int> indices;
      ToDimensionalIndices(s, &indices);
      LOG(ERROR) << "NULL Sample for: " << DebugIndicesString(indices);
    }
  }
  CHECK_EQ(0, null_samples);
}

int GridLookup::GetSampleIdx(int64 key, const Sample* sample) const {
  int index1 = PrecompLookup::GetSampleIdx(key, sample);
  vector<int> indices;
  for (int d = 0; d < n_dims(); ++d) {
    indices.push_back(IndexOfMatchingCut(*sample, d));
  }
  int index2 = ToSingleIndex(indices);

  if (index1 >= 0) {
    CHECK_EQ(index1, index2) << " for sample " << *sample << " with key: " << getSampleHash(*sample);
  }
  return index2;
}

void GridLookup::LogInfo() const {
  LOG(INFO) << "GRID_LOOKUP: " << samples_.size() << " samples; "
            << n_dims() << " dimensions.";
}

string GridLookup::DebugIndicesString(const vector<int>& indices) const {
  stringstream ss;
  ss << "(";
  for (int i = 0; i < indices.size(); ++i) {
    ss << indices[i];
    if (i < indices.size() - 1) {
      ss << ",";
    }
  }
  ss << ") --> (";
  for (int i = 0; i < indices.size(); ++i) {
    ss << GetDimension(i).DebugCutString(indices[i]);
    if (i < indices.size() - 1) {
      ss << ",";
    }
  }
  ss << ")";
  return ss.str();
}

void GridLookup::ToDimensionalIndices(
    const int index, vector<int>* indices) const {
  int rem = index;
  int divisor = 1;
  for (int d = 1; d < n_dims(); ++d) {
    divisor *= GetDimension(d).n_cuts();
  }
  indices->resize(n_dims());
  for (int dim_id = n_dims() - 1; dim_id >= 0; --dim_id) {
    div_t div_res = div(rem, divisor);
    (*indices)[dim_id] = div_res.quot;
    rem = div_res.rem;
    divisor /= GetDimension(dim_id).n_cuts();
  }
}

int GridLookup::ToSingleIndex(const vector<int>& indices) const {
  int res = 0;
  int mult = 1;
  for (int d = 0; d < indices.size(); ++d) {
    res += indices[d] * mult;
    mult *= GetDimension(d).n_cuts();
  }
  CHECK_LT(res, samples_.size());
  return res;
}

void GridLookup::LoopOverD(const int d, const vector<int>& smin_indices,
                           vector<Hypercuboid*>* cuboids) const {
  if (d >= n_dims()) {
    const Sample* smin = samples_[ToSingleIndex(smin_indices)];

    // TODO: HACK: REMOVE CODE DUPLICATION
    vector<int> smax_indices = smin_indices;
    stringstream ss;
    ss << "Creating grid hypercuboid: ";
    for (int d = 0; d < n_dims(); ++d) {
      ss << smin_indices[d] << ", ";
      smax_indices[d] += 1;
    }
    VLOG(3) << ss.str();
    const Sample* smax = samples_[ToSingleIndex(smax_indices)];

    vector<const Sample*> hypercube_samples;
    hypercube_samples.resize(pow(2, n_dims()), NULL);
    for (int idx = 0; idx < hypercube_samples.size(); ++idx) {
      vector<int> cube_indices;
      Hypercuboid::ToDimensionalIndices(n_dims(), idx, &cube_indices);

      vector<int> indices = smin_indices;
      for (int i = 0; i < cube_indices.size(); ++i) {
        if (cube_indices[i] == 1) {
          indices[i] = smax_indices[i];
        }
      }
      hypercube_samples[idx] = samples_[ToSingleIndex(indices)];
    }

    CHECK_EQ(hypercube_samples[0], smin);
    CHECK_EQ(hypercube_samples[hypercube_samples.size() - 1], smax);
    cuboids->push_back(new Hypercuboid(smin, smax, hypercube_samples));
    return;
  }

  for (int cut_id = 0; cut_id < n_cuts(d) - 1; ++cut_id) {
    vector<int> smin_copy = smin_indices;
    smin_copy.push_back(cut_id);
    LoopOverD(d + 1, smin_copy, cuboids);
  }
}

void GridLookup::AllCuboids(vector<Hypercuboid*>* cuboids) const {
  vector<int> smin;
  LoopOverD(0, smin, cuboids);
}

void GridLookup::CopyAllLeaves(vector<const Hypercuboid*>* cuboids) const {
  vector<Hypercuboid*> mut_cuboids;
  AllCuboids(&mut_cuboids);
  cuboids->insert(cuboids->begin(), mut_cuboids.begin(), mut_cuboids.end());
}

Hypercuboid GridLookup::FindContainingHypercuboid(const Sample& s) const {
  vector<int> smin_indices;  // into n-dimensional array
  for (int d = 0; d < n_dims(); ++d) {
    smin_indices.push_back(IndexOfLeftCut(s, d));
  }
  const Sample* smin = samples_[ToSingleIndex(smin_indices)];

  vector<int> smax_indices = smin_indices;
  for (int d = 0; d < n_dims(); ++d) {
    smax_indices[d] += 1;
  }
  const Sample* smax = samples_[ToSingleIndex(smax_indices)];

  vector<const Sample*> hypercube_samples;
  hypercube_samples.resize(pow(2, n_dims()), NULL);
  for (int idx = 0; idx < hypercube_samples.size(); ++idx) {
    vector<int> cube_indices;
    Hypercuboid::ToDimensionalIndices(n_dims(), idx, &cube_indices);

    vector<int> indices = smin_indices;
    for (int i = 0; i < cube_indices.size(); ++i) {
      if (cube_indices[i] == 1) {
        indices[i] = smax_indices[i];
      }
    }
    hypercube_samples[idx] = samples_[ToSingleIndex(indices)];
  }

  CHECK_EQ(hypercube_samples[0], smin);
  CHECK_EQ(hypercube_samples[hypercube_samples.size() - 1], smax);
  return Hypercuboid(smin, smax, hypercube_samples);
}

// HYBRID LOOKUP ---------------------------------------------------------------
HybridLookup::HybridLookup(const HybridLookupSpec& spec,
                           const FabGeometryTemplate* tpl)
    : PrecompLookup(new ControlValuesHelper(
        spec.grid_spec().control_names(), tpl->params_container())),
      grid_(new GridLookup(spec.grid_spec(), tpl)) {
  VLOG(2) << "Creating hybrid lookup";
  grid_->AllCuboids(&roots_lin_);
  VLOG(2) << "Creating " << roots_lin_.size() << " root cuboids from grid.";

  for (Hypercuboid* hc : roots_lin_) {
    roots_.insert(std::make_pair(hc->smin(), hc));
  }

  // Copy dimensions
  double_dims_ = grid_->double_dims_;
  int_dims_ = grid_->int_dims_;
  bool_dims_ = grid_->bool_dims_;

  AddSamples(spec.adaptive_samples_file());
  AddCuboids(spec);
  AddGeoDeltas(spec.geo_deltas());
  LogInfo();
}

int HybridLookup::n_leaves() const {
  int n = 0;
  for (const auto& root : roots_lin_) {
    n += root->n_leaves();
  }
  return n;
}

void HybridLookup::CopyAllLeaves(vector<const Hypercuboid*>* cuboids) const {
  for (const Hypercuboid* hc : roots_lin_) {
    set<std::pair<const Hypercuboid*, int> > leaves;
    hc->GetLeaves(&leaves);
    for (const auto& pr : leaves) {
      cuboids->push_back(new Hypercuboid(*(pr.first)));
    }
  }
}

void HybridLookup::GetLeafDepths(vector<int>* depths) const {
  depths->clear();

  vector<const Hypercuboid*> level_cuboids;
  level_cuboids.insert(level_cuboids.begin(),
                       roots_lin_.begin(), roots_lin_.end());
  while (!level_cuboids.empty()) {
    vector<const Hypercuboid*> tmp = level_cuboids;
    level_cuboids.clear();
    int n = 0;
    for (const Hypercuboid* hc : tmp) {
      if (!hc->is_leaf()) {
        level_cuboids.push_back(hc->left());
        level_cuboids.push_back(hc->right());
      } else {
        ++n;
      }
    }
    depths->push_back(n);
  }
}

void HybridLookup::LogInfo() const {
  grid_->LogInfo();

  LOG(INFO) << "HYBRID LOOKUP: " << samples_.size() << " samples; "
            << n_dims() << " dimensions; " << n_roots() << " roots; "
            << n_leaves() << " leaves";
  for (int d = 0; d < n_dims(); ++d) {
    GetDimension(d).LogInfo();
  }
  vector<int> depths;
  GetLeafDepths(&depths);
  for (int i = 0; i < depths.size(); ++i) {
    LOG(INFO) << "   " << depths[i] << " leaves at level " << i;
  }

  geo_deltas_.LogInfo();
}

const PropertySummary* HybridLookup::GetPropertiesExact(
    const Sample* value) const {
  const PropertySummary* res = grid_->GetPropertiesExact(value);
  if (!res) {
    res = PrecompLookup::GetPropertiesExact(value);
  }
  return res;
}

const Sample* HybridLookup::FindMatchingSample(int64 key) const {
  const Sample* res = grid_->FindMatchingSample(key);
  if (res) {
    return res;
  }
  res = PrecompLookup::FindMatchingSample(key);
  if (res) {
    return res;
  }

  return NULL;
}

bool HybridLookup::SubdivideOne(Hypercuboid* leaf,
                                const int dim,
                                vector<const Sample*>* new_samples) {
  if (dim < 0) {
    LOG(INFO) << "Invalid subdivision dimension";
    return false;
  }
  VLOG(3) << "Subdividing along dimension " << dim;

  boost::shared_ptr<ControlValuesThreshold> thresh;
  int d = dim;  // variable to aid indexing by type
  if (d < double_dims_.size()) {
    DimensionTpl<double>* dim =
        CHECK_NOTNULL(dynamic_cast<DimensionTpl<double>*>(
            const_cast<Dimension*>(&GetDimension(d))));
    int min_cut = dim->GetMatchingCut(leaf->smin()->doubles()[d]);
    int max_cut = dim->GetMatchingCut(leaf->smax()->doubles()[d]);
    VLOG(1) << "Subdividing in double " << d
            << " between cuts " << min_cut << ", " << max_cut
            << " for values " << leaf->smin()->doubles()[d]
            << ", " << leaf->smax()->doubles()[d];
    thresh = boost::shared_ptr<ControlValuesThreshold>(dim->SplitBetweenCuts(min_cut, max_cut));
  } else if (d < (double_dims_.size() + int_dims_.size())) {
    DimensionTpl<int32>* dim =
        CHECK_NOTNULL(dynamic_cast<DimensionTpl<int32>*>(
            const_cast<Dimension*>(&GetDimension(d))));
    d -= double_dims_.size();
    int min_cut = dim->GetMatchingCut(leaf->smin()->ints()[d]);
    int max_cut = dim->GetMatchingCut(leaf->smax()->ints()[d]);
    VLOG(1) << "Subdividing in int " << (d + double_dims_.size())
            << " between cuts " << min_cut << ", " << max_cut;
    thresh = boost::shared_ptr<ControlValuesThreshold>(dim->SplitBetweenCuts(min_cut, max_cut));
  }

  if (thresh == NULL) {
    VLOG(1) << "Dimension " << GetDimension(d).name() << " cannot be split futher.";
    return false;
  }
  leaf->Split(*this, thresh, new_samples);
  for (const Sample* s : *new_samples) {
    AddSample(s);
  }

  return true;
}

void HybridLookup::GetSamplesWithoutProperties(
    vector<const Sample*>* unsampled_pts) const {
  for (const Sample* s : samples_) {
    try {
      bool props = IsValidExact(s);
    } catch (runtime_error& e) {
      unsampled_pts->push_back(s);
    }
  }
}

void HybridLookup::AddCuboids(const HybridLookupSpec& spec) {
  for (int i = 0; i < spec.cuboid_size(); ++i) {
    const HybridLookupSpec::HypercuboidSpec& hspec = spec.cuboid(i);
    // All root cuboids are assumed to belong to the uniform grid, so we ignore
    // parts of the proto.
    VLOG(3) << "Reading cuboid " << i;
    Hypercuboid* root = roots_lin_[i];

    if (hspec.has_thresh()) {
      boost::shared_ptr<ControlValuesThreshold> thresh(
          new ControlValuesThreshold(hspec.thresh()));
      Hypercuboid* left = CuboidFromSpec(hspec.left());
      Hypercuboid* right = CuboidFromSpec(hspec.right());
      root->Split(thresh, left, right);
    }
  }
}

void HybridLookup::CuboidToSpec(const Hypercuboid& cuboid,
                                HybridLookupSpec::HypercuboidSpec* spec) const {
  VLOG(5) << "Writing cuboid " << cuboid;
  CHECK(cuboid.smin());
  CHECK(cuboid.smax());
  spec->set_min_sample_id(cv_helper_->GetEncoding(*cuboid.smin()));
  spec->set_max_sample_id(cv_helper_->GetEncoding(*cuboid.smax()));

  for (const Sample* s : cuboid.samples()) {
    CHECK(s);
    spec->add_sample_id(cv_helper_->GetEncoding(*s));
  }

  if (!cuboid.is_leaf()) {
    CHECK(cuboid.thresh());
    CHECK(cuboid.left());
    CHECK(cuboid.right());

    cuboid.thresh()->ToSpec(spec->mutable_thresh());
    CuboidToSpec(*cuboid.left(), spec->mutable_left());
    CuboidToSpec(*cuboid.right(), spec->mutable_right());
  }
  VLOG(5) << "Done writing cuboid " << cuboid;
}


void HybridLookup::CuboidsToSpec(HybridLookupSpec* spec) const {
  for (int i = 0; i < roots_lin_.size(); ++i) {
    VLOG(3) << "Writing cuboid: " << i;

    const Hypercuboid* root = CHECK_NOTNULL(roots_lin_[i]);
    HybridLookupSpec::HypercuboidSpec* root_spec = spec->add_cuboid();
    if (!root->is_leaf()) {

      CHECK(root->thresh());
      CHECK(root->left());
      CHECK(root->right());
      root->thresh()->ToSpec(root_spec->mutable_thresh());
      CuboidToSpec(*root->left(), root_spec->mutable_left());
      CuboidToSpec(*root->right(), root_spec->mutable_right());
    }
  }
}

Hypercuboid* HybridLookup::CuboidFromSpec(const HybridLookupSpec::HypercuboidSpec& spec) {
  vector<const Sample*> samples;
  for (const string& id : spec.sample_id()) {
    samples.push_back(FindMatchingSample(Sample(*cv_helper_, id)));
    CHECK(samples.back()) << "Could not find sample with id \"" << id << "\"";
  }

  Hypercuboid* res = new Hypercuboid(
      CHECK_NOTNULL(FindMatchingSample(Sample(*cv_helper_, spec.min_sample_id()))),
      CHECK_NOTNULL(FindMatchingSample(Sample(*cv_helper_, spec.max_sample_id()))),
      samples);

  if (spec.has_thresh()) {
    boost::shared_ptr<ControlValuesThreshold> thresh(
        new ControlValuesThreshold(spec.thresh()));
    Hypercuboid* left = CuboidFromSpec(spec.left());
    Hypercuboid* right = CuboidFromSpec(spec.right());
    res->Split(thresh, left, right);
  }

  return res;
}

HybridLookup::~HybridLookup() {
  for (Hypercuboid* hc : roots_lin_) {
    delete hc;
  }
}

Hypercuboid HybridLookup::FindContainingHypercuboid(const Sample& s) const {
  Hypercuboid grid_cell = grid_->FindContainingHypercuboid(s);
  VLOG(3) << "Found grid cell: " << grid_cell;

  map<const Sample*, Hypercuboid*>::const_iterator it =
      roots_.find(grid_cell.smin());
  if (it == roots_.end()) {
    LOG(ERROR) << " Could not find a corresponding cell; should never happen.";
    throw runtime_error("Failed to find cuboid corresponding to grid cuboid");
  }
  CHECK(it->second);
  VLOG(3) << "Found corresponding root: " << *it->second;
  return *it->second->Find(s);
}

// INDEXER ---------------------------------------------------------------------

SampleIndexer::DimIndexInfo::DimIndexInfo(const Dimension& dim, int bins) {
  start = dim.min_val();
  bin_size = dim.range() / bins;
}

SampleIndexer::SampleIndexer(const vector<const Dimension*>& dims)
    : bits_per_dim_(10),
      bins_per_dim_(512) {
  for (const Dimension* dim : dims) {
    infos_.push_back(DimIndexInfo(*dim, bins_per_dim_));
  }
}

int64 SampleIndexer::GetIndex(const ControlValues& sample) const {
  vector<double> values;
  sample.ToDoubleVector(&values);
  int64 res = 0;
  for (int i = 0; i < values.size(); ++i) {
    if (i > 0) {
      res = res << bits_per_dim_;
    }
    res += IndexInDim(values[i], i);
  }
  VLOG(6) << "Sample " << sample << " has index " << res;
  return res;
}

int SampleIndexer::IndexInDim(double val, int dim_id) const {
  const DimIndexInfo& dinfo = infos_[dim_id];

  int res = static_cast<int>(round((val - dinfo.start) / dinfo.bin_size));
  if (res < 0) {
    res = 0;
  } else if (res > bins_per_dim_) {
    res = bins_per_dim_;
  }
  VLOG(6) << "Index " << dim_id << " of " << val << " is " << res;
  return res;
}

}  // namespace mit_plato
