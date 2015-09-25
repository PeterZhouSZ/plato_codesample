#include <fstream>
#include <iostream>
using std::cin;
using std::cout;
using std::endl;

#include <boost/smart_ptr/scoped_ptr.hpp>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <google/protobuf/text_format.h>
#include <boost/filesystem.hpp>

#include "fab/geometry/template/template.h"
#include "fab/geometry/template/library.h"
#include "fab/geometry/template/params.h"
#include "fab/geometry/template/template.pb.h"
#include "util/proto_util.h"

#include <google/protobuf/stubs/common.h>
using google::protobuf::int32;

#include "fab/geometry/template/precomp/key_util.h"


DEFINE_string(tpl_filename, "", "");

DEFINE_string(output_file, "", "Samples output, one per line.");

DEFINE_string(control_names_out_file, "",
              "Control names, CSV for use with HadoopMapper.");

DEFINE_int32(n_divisions, 5,
             "Number of samples across each control axis.");

using mit_plato::FabGeometryTemplate;
using mit_plato::FabTemplateProto;
using mit_plato::VarHandle;
using mit_plato::OperationNode;
using namespace mit_plato;
namespace bfs = boost::filesystem;

class UniformGridSampler {
 public:
  UniformGridSampler(
      FabGeometryTemplate* tpl,
      std::fstream& output)
      : tpl_(tpl),
        output_(output),
        key_util_(new KeyUtil(*tpl)) {
    for (const auto& control : controls()) {
      VLOG(1) << "Control: " << control.name() << " [ "
              << VarInfoSpec::VarType_Name(control.info()->type()) << "]";
    }
  }

  const vector<TemplateParams::ControlInfo>& controls() const {
    return key_util_->controls();
  }

  bool Sample() {
    LoopOverControls(controls(), 0);
    output_.close();
    return output_.good();
  }

  string GetControlNames() const {
    return key_util_->ControlNames();
  }

  private:
  FabGeometryTemplate* tpl_;
  std::fstream& output_;
  const string file_prefix_;
  boost::scoped_ptr<KeyUtil> key_util_;

  void LoopOverControls(const vector<TemplateParams::ControlInfo>& controls,
                        const int i) {
    if (i >= controls.size()) {
      string encoded;
      key_util_->EncodeControlValues(tpl_, &encoded);
      output_ << encoded << "\n";
      return;
    }

    // Decide how to break the control
    const TemplateParams::ControlInfo& cinfo = controls[i];
    if (cinfo.info()->type() == VarInfoSpec::TYPE_INT32) {
      const IntBounds* int_bounds = CHECK_NOTNULL(
          dynamic_cast<const IntBounds*>(cinfo.bounds()));
      CHECK_EQ(1, int_bounds->intervals().size());
      int start_val = int_bounds->intervals()[0].min_val();
      int end_val = int_bounds->intervals()[0].max_val();
      int increment = fmax((end_val - start_val) / FLAGS_n_divisions, 1);
      for (int val = start_val; val <= end_val; val += increment) {
        VLOG(1) << "SETTING " << cinfo.info()->handle().short_name()
                  << " = " << val;
        CHECK(tpl_->SetParam<int32>(cinfo.info()->handle(), val));
        LoopOverControls(controls, i + 1);
      }
    } else if (cinfo.info()->type() == VarInfoSpec::TYPE_DOUBLE) {
      const DoubleBounds* double_bounds = CHECK_NOTNULL(
          dynamic_cast<const DoubleBounds*>(cinfo.bounds()));
      CHECK_EQ(1, double_bounds->intervals().size());
      double start_val = double_bounds->intervals()[0].min_val() + 0.0001;
      double end_val = double_bounds->intervals()[0].max_val() - 0.0001;
      double increment = (end_val - start_val) / FLAGS_n_divisions;
      double val = start_val;
      for (int k = 0; k <= FLAGS_n_divisions; ++k) {
        if (val > end_val) {
          val = end_val;
        }
        VLOG(1) << "SETTING " << cinfo.info()->handle().short_name()
                  << " = " << val;
        CHECK(tpl_->SetParam<double>(cinfo.info()->handle(), val));
        LoopOverControls(controls, i + 1);
        val += increment;
      }
    } else if (cinfo.info()->type() == VarInfoSpec::TYPE_BOOL) {
      VLOG(1) << "SETTING " << cinfo.info()->handle().short_name()
                << " = true";
      CHECK(tpl_->SetParam<bool>(cinfo.info()->handle(), true));
      LoopOverControls(controls, i + 1);

      VLOG(1) << "SETTING " << cinfo.info()->handle().short_name()
                << " = false";
      CHECK(tpl_->SetParam<bool>(cinfo.info()->handle(), false));
      LoopOverControls(controls, i + 1);
    } else {
      LOG(WARNING) << "Cannot loop over control type: "
                   << VarInfoSpec::VarType_Name(cinfo.info()->type());
      LoopOverControls(controls, i + 1);
    }
  }
};

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  FLAGS_logtostderr = true;

  CHECK(!FLAGS_output_file.empty());
  CHECK(!FLAGS_control_names_out_file.empty());

  if (!mit_plato::FabTemplateLibrary::InitLibrary()) {
    LOG(ERROR) << "Failed to init library!";
  }

  boost::scoped_ptr<FabGeometryTemplate> tpl(
      FabGeometryTemplate::CheckCreateFromFile(FLAGS_tpl_filename));

  // Get all the controls
  std::fstream output(
      FLAGS_output_file.c_str(),
      std::fstream::out | std::fstream::trunc);
  UniformGridSampler sampler(tpl.get(), output);
  CHECK(sampler.Sample());

  std::fstream control_names_output(
      FLAGS_control_names_out_file.c_str(),
      std::fstream::out | std::fstream::trunc);
  control_names_output << sampler.GetControlNames();
  control_names_output.close();
  CHECK(control_names_output.good());

  return 0;
}
