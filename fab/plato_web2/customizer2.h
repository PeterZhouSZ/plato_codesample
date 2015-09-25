#ifndef _FAB_PLATO_WEB2_CUSTOMIZER2_H__
#define _FAB_PLATO_WEB2_CUSTOMIZER2_H__

#include <iostream>
#include <map>
#include <string>
#include <vector>

#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/function.hpp>
#include <boost/smart_ptr/scoped_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include "fab/geometry/template/template.h"
#include "fab/geometry/template/precomp/lookup.h"


namespace mit_plato {

class CustomizerSpec;

namespace web {

using std::string;

class TemplateCustomizer2;


class Customizer2Factory {
 public:
  static void Init();

  static TemplateCustomizer2* FromName(const string& name);

  static TemplateCustomizer2* FromFile(const string& rel_filename);

  static TemplateCustomizer2* FromSpec(const CustomizerSpec& spec,
                                       const string& cache_key = "");

  static string GetLogFilename(const string& tpl_filename);

 private:
  Customizer2Factory() {}
};


// Main class used by the Web server to interact with a geometry template with
// a precomputation table.
class TemplateCustomizer2 {
 public:
  virtual ~TemplateCustomizer2() {}

  // Returns template information as follows:
  //
  //   seeds: [ "1.5_6", "2.0_3" ],
  //   controls: [
  //     { name: "width",
  //       range: [ 1, 20 ],
  //       type: "INT" | "DOUBLE" | "BOOL",
  //       value: 3.5
  //      }, ... ],
  //   name: "Template Name"
  void EncodeDesignInfo(std::ostream& info);

  // Input format:
  // control_name value control_name2 value1 value2
  // Output format:
  // {"control_name": [ [start1, end1], [start2, end2] ] }, ...]}
  virtual bool SetFromStream(std::istream& istream,
                             std::ostream& changed_bounds_ostream);

  typedef boost::function<void (const string& encoded_message)> CallbackFunc;

  // Sets callback to call whenever new control values result
  // in new geometry preview; generates initial geometry and calls the callback
  // on it.
  virtual void SetUpdatedGeometryCallback(CallbackFunc callback);

  // Input format:
  // control_name1 control_name2
  // Output:
  // encoded valid grid
  virtual bool GetValidGrid2D(std::istream& control_names,
                              std::ostream& valid_grid_ostream);


  // EXTRANEOUS ADDITIONS TO CLEAN INTERFACE -----------------------------------
  const PrecompLookup* lookup() const { return lookup_; }

  // Note: of course this method should not be public
  void UpdateGeometry(ControlValues value);

  void SetLogFile(const string& logfile);
  void SetLoggingEnabled(bool enable) { logging_on_ = enable; }

 private:
  TemplateCustomizer2(FabGeometryTemplate* tpl,
                      PrecompLookup* lookup = NULL,
                      GeometryLookup* geo_cache = NULL);

  const TemplateParams& params() const;

  ControlValues GetPreviewPoint(ControlValues current);
  void SendPreview();

  void UpdatePreviewRequest(ControlValues current);
  void FillLatestPreviewRequest();
  void SendLatestPreviewUpdate(ControlValues current);

  // High level function
  void SendPreviewUpdate(ControlValues current);

  void ToJSON(const TemplateParams::ControlInfo& info, std::ostream& os);
  void BoundIntervalsToJson(const vector<Bounds*>& bounds,
                            const set<string> to_exclude,
                            std::ostream& os);
  void FillLookupInfo(ControlValues current,
                      const set<string>& excluded_controls,
                      std::ostream& update_ostream);

  bool LoggingIsOn() const;
  double GetTimeMs() const;
  void MaybeLog(const string& command, const Sample* s = NULL);
  void MaybeLog(const string& command, const string& args);

  // Base operations
  // Determine if new preview is needed

  // Current setting of control values
  ControlValues current_;

  boost::mutex preview_lock_;
  ControlValues pending_preview_;

  boost::mutex geo_lock_;
  ControlValues last_preview_;
  boost::scoped_ptr<FabGeometryTemplate> tpl_;

  CallbackFunc geo_callback_;
  std::vector<TemplateParams::ControlInfo> controls_;
  ControlValuesHelper values_helper_;

  PrecompLookup* lookup_;
  GeometryLookup* cache_;

  std::vector<string> seeds_;
  LookupStatsProto stats_spec_;

  // Hacky way to log all operations
  bool logging_on_;
  std::ofstream command_log_stream_;
  boost::posix_time::ptime start_time_;

  friend class Customizer2Factory;
};

// encodes grid as
//
// 32-bit int for X_PTS
// 32-bit int for Y_PTS
// X_PTS * Y_PTS * 2 32-bit floats for x and y values of each
// point, row-major
// X_PTS * Y_PTS 16-bit ints for the failure code
class GridEncoder {
 public:
  // static void EncodeAsString(
  //     const ValidityGrid2D& grid,
  //     string* s) {
  //   int metadata[2] = {grid.x_dim(), grid.y_dim()};

  //   size_t meta_size = sizeof metadata;
  //   size_t pos_size = sizeof(float) *  grid.x_dim() * grid.y_dim() * 2;
  //   size_t code_size = sizeof(short int) * grid.x_dim() * grid.y_dim();

  //   s->resize(meta_size + pos_size + code_size);
  //   size_t write_pos = start_pos;

  //   {
  //     char const * p_metadata = reinterpret_cast<char const *>(metadata);
  //     std::copy(p_metadata, p_metadata + meta_size, &((*s)[write_pos]));
  //     write_pos += meta_size;
  //   }

  //   {
  //     char const * p_positions = reinterpret_cast<char const *>(grid.raw_positions());
  //     std::copy(p_positions, p_positions + pos_size, &((*s)[write_pos]));
  //     write_pos += pos_size;
  //   }

  //   {
  //     char const * p_code = reinterpret_cast<char const *>(grid.raw_codes());
  //     std::copy(p_code, p_code + code_size, &((*s)[write_pos]));
  //   }
  // }

  static void EncodeAsJson(
      const ValidityGrid2D& grid,
      std::ostream& out) {
    out << "{x_dim:" << grid.x_dim() << ",y_dim:" << grid.y_dim() << ",";

    out << "pos: [" << std::setprecision(4);
    for (int i = 0; i < grid.positions_.size(); ++i) {
      if (i > 0) {
        out << ",";
      }
      out << grid.positions_[i];
    }
    out << "],";

    out << "codes: [";
    for (int i = 0; i < grid.failure_codes_.size(); ++i) {
      if (i > 0) {
        out << ",";
      }
      out << grid.failure_codes_[i];
    }
    out << "]}";
  }
};

}  // namespace web
}  // namespace mit_plato

#endif  // _FAB_PLATO_WEB2_CUSTOMIZER2_H__
