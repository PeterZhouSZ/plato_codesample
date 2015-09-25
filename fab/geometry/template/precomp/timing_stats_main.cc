#include <string>
using std::string;
#include <vector>
using std::vector;

#include <fstream>
#include <iostream>
using std::cout;
using std::cin;
using std::endl;

#include <boost/timer/timer.hpp>
#include <boost/smart_ptr/scoped_ptr.hpp>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include "fab/geometry/template/customizer.h"
#include "fab/geometry/template/precomp/lookup.h"
#include "fab/geometry/template/library.h"
#include "fab/plato_web2/customizer2.h"
#include "fab/geometry/template/precomp/stats.h"
#include "fab/geometry/template/template.pb.h"
#include "util/proto_util.h"

DEFINE_string(customizer_spec, "",
              "Location of the appropriate CustomizerSpec proto.");

DEFINE_string(samples_in_out, "",
              "Text file to read samples to/ write samples to.");

DEFINE_string(stats_out, "",
              "File to write stats to.");

DEFINE_int32(num_samples, 200,
             "Number of samples to generate, if any.");

DEFINE_string(caching_strategy, "OPT",
              "Use OPT, NONE, ALL.");

using namespace mit_plato;

using namespace mit_plato::web;

void GenerateSamples(const PrecompLookup& lookup, vector<string>* samples) {
  LookupStats stats(lookup);

  for (int i = 0; i < FLAGS_num_samples + 50; ++i) {
    LookupStats::VertexDesc vd =
        boost::vertex(rand() % boost::num_vertices(stats.graph()), stats.graph());
    samples->push_back(lookup.cv_helper().GetEncoding(*CHECK_NOTNULL(stats.graph()[vd].sample)));
  }
}

void ReadSamples(vector<string>* samples) {
  std::ifstream infile(FLAGS_samples_in_out.c_str());
  std::string line;
  while (std::getline(infile, line)) {
    samples->push_back(line);
  }
  infile.close();
}

void WriteSamples(const vector<string>& samples) {
  std::ofstream ofile;
  ofile.open(FLAGS_samples_in_out.c_str(), std::fstream::out | std::fstream::trunc);
  for (const string& sample : samples) {
    ofile << sample << "\n";
  }
  ofile.close();
}

void GetSamples(const PrecompLookup& lookup, vector<string>* samples) {
  ReadSamples(samples);
  if (samples->empty()) {
    LOG(INFO) << "No samples in file; generating and writing";
    GenerateSamples(lookup, samples);
    WriteSamples(*samples);
  }
}

namespace {
double GetTimerMs(boost::timer::cpu_timer& timer) {
  auto nanoseconds = boost::chrono::nanoseconds(timer.elapsed().user + timer.elapsed().system);
  auto milliseconds = boost::chrono::duration_cast<boost::chrono::milliseconds>(nanoseconds);
  return milliseconds.count();
}
}

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (!mit_plato::FabTemplateLibrary::InitLibrary()) {
    LOG(ERROR) << "Failed to init library!";
  }

  CHECK(!FLAGS_samples_in_out.empty());
  CHECK(!FLAGS_customizer_spec.empty());

  // Open customizer
  CustomizerSpec spec;
  CHECK(mds_util::ReadProtoFromFile(FLAGS_customizer_spec, &spec))
      << "Failed to parse CustomizerSpec from file " << FLAGS_customizer_spec;
  if (FLAGS_caching_strategy == "NONE") {
    LOG(INFO) << "Disabling caching";
    spec.clear_subtrees_lookup_filename();
  } else if (FLAGS_caching_strategy == "OPT") {
    CustomizerFactory::SetResourceCaching(true);
  } else if (FLAGS_caching_strategy == "ALL") {
    CustomizerFactory::SetResourceCaching(true);
    CustomizerFactory::SetCacheOptimization(false);
  } else {
    LOG(FATAL) << "Unknown caching strategy \"" << FLAGS_caching_strategy << "\"";
  }
  boost::scoped_ptr<TemplateCustomizer2> customizer(
      dynamic_cast<TemplateCustomizer2*>(Customizer2Factory::FromSpec(spec)));
  // ControlValuesHelper helper(*(customizer->DebugTpl()));

  // Get (or generate) samples
  vector<string> samples;
  GetSamples(*customizer->lookup(), &samples);

  if (FLAGS_stats_out.empty()) {
    LOG(INFO) << "Stats file empty; not generating any";
    return 0;
  }

  // Open output
  std::ofstream output;
  output.open(FLAGS_stats_out.c_str(), std::fstream::out | std::fstream::trunc);
  CHECK(output.good()) << "Failed to open " << FLAGS_stats_out;

  // Get stats
  int successes = 0;
  for (int i = 0; i < samples.size(); ++i) {
    const string& sample_str = samples[i];
    LOG(INFO) << "Evaluating " << sample_str << " (i = " << i << ")";

    Sample s(customizer->lookup()->cv_helper(), sample_str);
    boost::timer::cpu_timer timer;
    try {
      customizer->UpdateGeometry(s);
      ++successes;
    } catch (...) {
      LOG(ERROR) << "Failed to generate geometry";
      continue;
    }
    double ms = GetTimerMs(timer);
    output << sample_str << " " << ms << "\n";
    output.flush();
    if (successes == FLAGS_num_samples) {
      break;
    }
  }
  output.close();
  LOG(INFO) << "Wrote stats to: " << FLAGS_stats_out;

  return 0;
}
