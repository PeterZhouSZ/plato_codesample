#ifndef _FAB_GEOMETRY_TEMPLATE_TEMPLATE_PRECOMP_KEY_UTIL_H__
#define _FAB_GEOMETRY_TEMPLATE_TEMPLATE_PRECOMP_KEY_UTIL_H__

#include <set>
#include <string>
#include <vector>
#include <iostream>
#include <utility>

#include "fab/geometry/template/template_params.h"
#include "fab/geometry/template/template.h"

namespace mit_plato {

using std::string;
using std::vector;
using std::istream;
using std::ostream;
using std::stringstream;

class ControlValues;
class GeometryLookup;

class KeyUtil {
 public:
  KeyUtil(const FabGeometryTemplate& tpl);

  KeyUtil(const string& control_names,
          const TemplateParams& params);

  // Returns csv string of controls suitable for instantiating
  // KeyUtil
  string ControlNames() const;

  // TODO: allow encoding a range of values
  void SetControls(const string& values,
                   FabGeometryTemplate* tpl,
                   string* file_suffix = NULL) const;

  void SetControls(const ControlValues& values,
                   FabGeometryTemplate* tpl,
                   string* file_suffix = NULL) const;

  // Same result as file_suffix above
  string EncodeValue(const ControlValues& v) const;

  // Truncates double values in the encoded value for easier
  // matches
  string GetKey(const ControlValues& v) const;

  static string EncodingToKey(const string& encoding);

  // Given currently set controls, run solver,
  // marks subrees as dirty (if requested),
  // inserts chached subtrees (if requested),
  // and updated geometry
  static void UpdateTpl(FabGeometryTemplate* tpl,
                        const GeometryLookup* geo_cache = NULL,
                        bool fresh_recompute = false);

  // Returns true if properties OK
  void ComputeProperties(FabGeometryTemplate* tpl,
                         const string& property_key,
                         PropertyValuesSpec* spec,
                         const GeometryLookup* geo_cache = NULL,
                         bool fresh_recompute = false) const;

  void EncodeProperties(const PropertyValuesSpec& spec,
                        string* encoded) const;

  // Encodes controls in a format that can be used to run SetControls
  void EncodeControlValues(FabGeometryTemplate* tpl,
                           string* encoded) const;

  // Get information about subtrees that should be cached;
  // should be called after CheckEncodeProperties
  void ChooseCachedGeometryNodes(
      FabGeometryTemplate* tpl,
      const string& mesh_file_prefix,
      const string& mesh_file_format,
      set<std::pair<SubtreeCachingInfo, const OperationNode* > >* cached_geo) const;

  const vector<TemplateParams::ControlInfo>& controls() const { return controls_; }

  template <class T>
  void MaybeSetValue(const T& new_value,
                     const VarHandle& handle,
                     FabGeometryTemplate* tpl) const;

  template <class T>
  static T ReadOrThrow(stringstream& ss);

  template <class T>
  static bool AreTwoEqual(const T& v1, const T& v2) {
    return v1 == v2;
  }

 private:
  static void UpdateTplGeometry(FabGeometryTemplate* tpl,
                                const GeometryLookup* geo_cache,
                                bool fresh_recompute);

  template <class T, class TSET = T>
  void MaybeSet(stringstream& ss,
                ostream& os,
                const VarHandle& handle,
                FabGeometryTemplate* tpl) const;

  template <class T>
  void Encode(ostream& os, const T& val) const {
    os << val;
  }

  vector<TemplateParams::ControlInfo> controls_;
};

template <>
bool KeyUtil::AreTwoEqual<double>(const double& v1,
                                            const double& v2);

template <>
void KeyUtil::Encode<double>(ostream& os, const double& val) const;


template <class T>
T KeyUtil::ReadOrThrow(stringstream& ss) {
  T val = 0;
  ss >> val;
  if (ss.fail()) {
    throw runtime_error(
        "Failed to read expected type from stream " + ss.str());
  }
  return val;
}


template <class T>
void KeyUtil::MaybeSetValue(const T& val,
                            const VarHandle& handle,
                            FabGeometryTemplate* tpl) const {
  VLOG(2) << "Setting " << handle << " to " << val;
  T current_val = tpl->GetParam<T>(handle);
  if (!AreTwoEqual<T>(val, current_val)) {
    if (!tpl->SetParam<T>(handle, val)) {
      throw runtime_error("Could not set " + handle.full_name());
    }
    VLOG(2) << "Set ok";
  }
}

template <class T, class TSET>
void KeyUtil::MaybeSet(stringstream& ss,
                                 ostream& os,
                                 const VarHandle& handle,
                                 FabGeometryTemplate* tpl) const {
  T val = ReadOrThrow<T>(ss);
  Encode<T>(os, val);
  MaybeSetValue<TSET>(val, handle, tpl);
}

inline bool operator<(const SubtreeCachingInfo& i1, const SubtreeCachingInfo& i2) {
  return i1.file() < i2.file();
}

} // namespace mit_plato

#endif
