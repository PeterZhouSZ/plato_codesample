#include <iostream>
#include <sstream>
#include <string>
using std::stringstream;

#include <boost/regex.hpp>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "fab/geometry/template/template.h"
#include "fab/geometry/template/precomp/lookup.h"
#include "fab/geometry/template/template.pb.h"
#include "fab/geometry/template/library.h"
#include "util/proto_util.h"


DEFINE_string(optimizer_type, "OPT_AVE",
              "Use NAIVE for naive, OPT_AVE for optimizing average time, "
              "and OPT_MAX for optimizing maximum time.");

DEFINE_bool(exploration_mode, true,
            "If true, does not output optimized cache, but the stats for graph.");
DEFINE_double(max_memory, 1000,
              "Max memory to optimize to (ignored in exploration mode)");
DEFINE_string(optimizer_output, "",
              "If not set, optimizer does not run. "
              "File to write output of optimization to.");

// Geometry graph construction (Or, read from file)
DEFINE_string(tpl_filename, "",
              "Tempalte filename.");
DEFINE_string(hybrid_lookup_spec, "",
              "Hybrid lookup filename");
DEFINE_string(aggregated_geo_cache, "",
              "File to read mapping from subtree key to geometry file.");
DEFINE_string(graph_output, "",
              "If non-empty, reads graph from file. "
              "Otherwise, outputs graph here.");

using namespace mit_plato;

GeometryDependencyGraph* CreateDepGraph() {
  LOG(INFO) << "Constructing dependency graph for geometry in all samples...";
  CHECK(!FLAGS_tpl_filename.empty() &&
        !FLAGS_aggregated_geo_cache.empty() &&
        !FLAGS_hybrid_lookup_spec.empty());

  std::ofstream debug_log;
  debug_log.open("/tmp/graph_diagnostics.txt",
                 std::fstream::out | std::fstream::trunc);

  boost::scoped_ptr<FabGeometryTemplate> tpl(
      FabGeometryTemplate::CheckCreateFromFile(FLAGS_tpl_filename));
  KeyUtil key_util(*tpl);
  LOG(INFO) << "Created template";

  SubtreesCacheSpec in_spec;
  CHECK(mds_util::ReadProtoFromFile(FLAGS_aggregated_geo_cache, &in_spec));
  GeometryLookup lookup(in_spec, "");
  LOG(INFO) << "Read cache spec from file and created GeometryLookup";

  HybridLookupSpec sample_lookup_spec;
  CHECK(mds_util::ReadProtoFromFile(FLAGS_hybrid_lookup_spec, &sample_lookup_spec));
  boost::scoped_ptr<HybridLookup> sample_lookup(
      new HybridLookup(sample_lookup_spec, tpl.get()));
  sample_lookup->LogInfo();

  GeometryDependencyGraph* dgraph = new GeometryDependencyGraph();
  int valid_samples = 0;
  for (int sample_id = 0; sample_id < sample_lookup->n_samples(); ++sample_id) {
    const Sample* s = sample_lookup->getSampleAt(sample_id);
    if (!sample_lookup->IsValidExact(s)) { continue; }

    string controls_str;
    sample_lookup->cv_helper().key_util().SetControls(*s, tpl.get(), &controls_str);

    try {
      tpl->RunSolver();
    } catch (runtime_error& e) {
      LOG(ERROR) << "Failed to get solution: " << e.what();
      continue;
      // throw runtime_error(string("Failed to get solution: ") + e.what());
    }
    ++valid_samples;
    tpl->RefreshNodeKeys();
    dgraph->AddToGraph(*tpl, controls_str);

    debug_log << *s << " --> " << dgraph->StatsString() << "\n";
    debug_log.flush();
  }
  dgraph->AddMetadata(in_spec);
  LOG(INFO) << "Built graph from " << valid_samples << " valid samples "
            << "(log: /tmp/graph_diagnostics.txt)";
  LOG(INFO) << "Valid samples stored: " << dgraph->valid_samples();
  dgraph->LogStats();
  CHECK_GT(valid_samples, 0) << "Something went terribly wrong";
  return dgraph;
}


void LogOptimizerStats(const CacheOptimizer& optimizer,
                       std::ofstream& output_stream) {
  output_stream << optimizer.memory() << " "
                << optimizer.ave_compute_time() << " "
                << optimizer.stdev_compute_time() << " "
                << optimizer.worst_compute_time() << " "
                << optimizer.total_compute_time() << " "
                << optimizer.num_cached_nodes() << " "
                << optimizer.num_touched_samples() << "\n";
  output_stream.flush();
}


int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (!mit_plato::FabTemplateLibrary::InitLibrary()) {
    LOG(ERROR) << "Failed to init library!";
  }

  CHECK(!FLAGS_graph_output.empty() ||
        !FLAGS_optimizer_output.empty()) << "No output specified.";

  boost::scoped_ptr<GeometryDependencyGraph> dgraph;
  if (!FLAGS_graph_output.empty()) {
    GeoDependencyGraphSpec gspec;
    if (mds_util::ReadProtoFromFile(FLAGS_graph_output, &gspec) &&
        gspec.node_size() > 0) {
      LOG(INFO) << "Read graph from input file: " << FLAGS_graph_output;
      dgraph.reset(new GeometryDependencyGraph(gspec));
      dgraph->LogStats();
    }
  }

  if (dgraph == NULL) {
    dgraph.reset(CreateDepGraph());

    if (!FLAGS_graph_output.empty()) {
      GeoDependencyGraphSpec gspec;
      dgraph->ToSpec(&gspec);
      mds_util::WriteASCIIProtoToFile(FLAGS_graph_output, gspec);
      LOG(INFO) << "Wrote graph to: " << FLAGS_graph_output;
    }
  }

  if (FLAGS_optimizer_output.empty()) {
    LOG(INFO) << "Skipping optimization; no --optimizer_output specified";
    return 0;
  }

  LOG(INFO) << "Running optimizer";
  std::ofstream output_stream;
  output_stream.open(FLAGS_optimizer_output.c_str(),
                     std::fstream::out | std::fstream::trunc);
  if (!output_stream) {
    LOG(FATAL) << "Failed to open output: " << FLAGS_optimizer_output;
  }

  CacheOptimizer optimizer(*dgraph);
  if (FLAGS_exploration_mode) {
    if (FLAGS_optimizer_type == "OPT_AVE" ||
        FLAGS_optimizer_type == "NAIVE") {
      while (!optimizer.AllCached()) {
        if (FLAGS_optimizer_type == "NAIVE") {
          optimizer.NaiveCacheNext();
        } else {
          optimizer.GreedyIterationAveTime();
        }
        LogOptimizerStats(optimizer, output_stream);
      }
    } else if (FLAGS_optimizer_type == "OPT_MAX") {
      while (optimizer.worst_compute_time() > 0.5 /*ms*/) {
        optimizer.GreedyIterationMaxTime();
        LogOptimizerStats(optimizer, output_stream);
      }
    } else {
      LOG(FATAL) << "Unknown optimizer type: " << FLAGS_optimizer_type;
    }
  } else {  // not exploration mode
    if (FLAGS_optimizer_type == "OPT_AVE") {
      while (optimizer.memory() < FLAGS_max_memory && !optimizer.AllCached()) {
        optimizer.GreedyIterationAveTime();
        LOG(INFO) << optimizer.memory() << " "
                  << optimizer.total_compute_time() << " "
                  << optimizer.num_cached_nodes() << " "
                  << optimizer.num_touched_samples() << "\n";
      }
    } else if (FLAGS_optimizer_type == "OPT_MAX") {
      while (optimizer.memory() < FLAGS_max_memory &&
             optimizer.worst_compute_time() > 0.5 /*ms*/) {
        optimizer.GreedyIterationMaxTime();
        LOG(INFO) << optimizer.memory() << " "
                  << optimizer.total_compute_time() << " "
                  << optimizer.num_cached_nodes() << " "
                  << optimizer.num_touched_samples() << "\n";
      }
    } else {
      LOG(FATAL) << "Unknown optimizer type: " << FLAGS_optimizer_type;
    }
    optimizer.WriteUsedKeys(output_stream);
    output_stream.flush();
  }

  output_stream.close();
  LOG(INFO) << "Done optimizing";
  return 0;
}
