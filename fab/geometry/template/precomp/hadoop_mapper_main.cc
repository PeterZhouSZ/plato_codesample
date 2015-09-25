#include <string>
#include <stdlib.h>
#include <iostream>
#include <sstream>

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/IO/Options.hh>
#include <boost/filesystem.hpp>
#include <boost/smart_ptr/scoped_ptr.hpp>
#include <gflags/gflags.h>
#include <glog/logging.h>

#include "fab/geometry/template/precomp/aws_util.h"
#include "fab/geometry/template/precomp/key_util.h"
#include "fab/geometry/template/precomp/lookup.h"
#include "fab/geometry/template/precomp/sampling_mapper.h"
#include "fab/geometry/template/template.h"
#include "fab/geometry/template/template.pb.h"
#include "fab/geometry/template/library.h"
#include "util/proto_util.h"

// Common to ALL
DEFINE_string(control_names, "",
              "Control names used to generate control samples in map input.");

// Common to ALL
DEFINE_string(tpl_filename,
              "",
              "Template filename to load; must be FabTemplateProto message "
              "in protocol buffer ascii or binary format.");

// Common to ALL
DEFINE_string(local_dir, "/tmp",
              "Local directory to use for storing local files before moving "
              "to S3.");

// Specific to ADAPT
DEFINE_string(s3_snapshot_dir, "",
              "If set, is used to copy hybrid_lookup_specs previously written "
              "to safeguard against disk caching or other I/O overwrite issues.");

// Specific to UNI and ADAPT
DEFINE_string(geometry_out_dir, "",
              "If set, will output sampled geometry; typically on AWS.");

DEFINE_string(local_geometry_out_dir, "",
              "Local gometry output directory.");

// Specific to UNI and ADAPT
DEFINE_string(samples_out_file, "",
              "File to write encoded properties to.");

// Specific to UNI and ADAPT
DEFINE_string(geometry_index_out_file, "",
              "File to write mapping from subtree key to geometry file.");

// Specific to UNI and ADAPT
DEFINE_bool(restart, false,
            "If true, will read the output files and append to them "
            "and will skip the work previously done.");

// Specific to ADAPT
DEFINE_string(hybrid_lookup_spec, "",
              "--action=ADAPT: Spec of the format HybridLookupSpec.");

DEFINE_int32(cuboid_id, 0,
             "--action=ADAPT: Defines the start hypercuboid id for this worker");

DEFINE_int32(cuboids_to_process, 1,
             "--action=ADAPT: Number of root cuboids to process; "
             "if negative --> all will be processed.");

DEFINE_int32(max_depth, 2,
             "--action=ADAPT: Maximum depth of the KD tree.");

DEFINE_string(action, "UNI", "Uniform sample, else ADAPT.");


DECLARE_string(aws_access_key);
DECLARE_string(aws_secret_key);
DECLARE_bool(enable_triangle_logging);
DECLARE_bool(use_geometry_deltas);
DECLARE_double(min_split_geo_delta);

using namespace mit_plato;
using std::stringstream;
namespace bfs = boost::filesystem;

string GetSnapshotFilename() {
  stringstream ss;
  ss << FLAGS_s3_snapshot_dir << "/"
     << bfs::basename(FLAGS_hybrid_lookup_spec) << ".SNAP"
     << rand() % 100000;
  return ss.str();
}

string BinaryHybridLookupFilename() {
  return FLAGS_hybrid_lookup_spec + ".binary";
}

void BackupHybridLookup(const HybridLookup& lookup,
                        HybridLookupSpec* lookup_spec) {
  // We save previous snapshots to S3
  if (!FLAGS_s3_snapshot_dir.empty()) {
    const string snap_filename = GetSnapshotFilename();
    if (Aws::Cp(FLAGS_hybrid_lookup_spec, snap_filename)) {
      VLOG(1) << "Copied to snapshot " << snap_filename;
    } else {
      LOG(ERROR) << "Failed to copy to snapshot " << snap_filename;
    }
    Aws::Cp(BinaryHybridLookupFilename(), snap_filename + ".binary");
  }

  lookup_spec->clear_cuboid();
  lookup_spec->clear_geo_deltas();

  lookup.CuboidsToSpec(lookup_spec);
  lookup.GeoDeltasToSpec(lookup_spec->mutable_geo_deltas());

  // Write binary (much shorter)
  const string& binary_filename = BinaryHybridLookupFilename();
  if (!mds_util::SerializeProtoToFile(binary_filename, *lookup_spec)) {
    LOG(ERROR) << "Failed to write lookup spec to file " << binary_filename;
  } else {
    VLOG(1) << "Backed up HybridLookup to (BINARY) " << binary_filename;
  }

  // Write ascii (readable)
  if (!mds_util::WriteASCIIProtoToFile(
          FLAGS_hybrid_lookup_spec, *lookup_spec)) {
    LOG(ERROR) << "Failed to write lookup spec to file "
               << FLAGS_hybrid_lookup_spec;
  } else {
    VLOG(1) << "Backed up HybridLookup to (ASCII) " << FLAGS_hybrid_lookup_spec;
  }
}

HybridLookup* LoadHybridLookupFromFile(HybridLookupSpec* lookup_spec,
                                       FabGeometryTemplate* tpl,
                                       const GeometryLookup* geo_cache,
                                       std::ofstream* deltas_stream) {
  LOG(INFO) << "> Reading HybridLookupSpec from " << FLAGS_hybrid_lookup_spec;
  if (!mds_util::ReadProtoFromFile(
          FLAGS_hybrid_lookup_spec, lookup_spec)) {
    LOG(ERROR) << "Failed to parse lookup spec from file "
               << FLAGS_hybrid_lookup_spec;

    // Try reading from binary instead
    const string& binary_filename = BinaryHybridLookupFilename();
    CHECK(mds_util::ReadProtoFromFile(
        binary_filename, lookup_spec))
        << "Failed to read from " << binary_filename;
    CHECK(!lookup_spec->grid_spec().control_names().empty())
        << "Read empty spec from " << binary_filename << ": "
        << lookup_spec->DebugString();
  }

  lookup_spec->set_adaptive_samples_file(FLAGS_samples_out_file);
  LOG(INFO) << "About to run constructor";
  HybridLookup* lookup = new HybridLookup(*lookup_spec, tpl);

  // Let lookup compute geometry delta while relying on the updated cache
  lookup->geo_delta_lookup()->EnableComputation(geo_cache);
  lookup->geo_delta_lookup()->EnableBackup(deltas_stream);

  LOG(INFO) << "> Created Hybrid Lookup: ";
  lookup->LogInfo();
  return lookup;
}

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (!mit_plato::FabTemplateLibrary::InitLibrary()) {
    LOG(ERROR) << "Failed to init library!";
  }

  CHECK(FLAGS_action == "UNI" ||
        FLAGS_action == "ADAPT");

  CHECK(!FLAGS_control_names.empty());
  CHECK(!FLAGS_tpl_filename.empty());
  CHECK(!FLAGS_local_dir.empty());
  CHECK(!FLAGS_aws_access_key.empty());
  CHECK(!FLAGS_aws_secret_key.empty());
  CHECK(FLAGS_geometry_out_dir.empty() ||
        !FLAGS_geometry_index_out_file.empty());
  CHECK_EQ(FLAGS_geometry_out_dir.empty(),
           FLAGS_local_geometry_out_dir.empty());
  CHECK(!FLAGS_samples_out_file.empty());

  CHECK(FLAGS_action == "UNI" ||
        !FLAGS_hybrid_lookup_spec.empty());

  const string local_tpl_file = FLAGS_local_dir + "/template.ascii_proto";
  CHECK(Aws::Cp(FLAGS_tpl_filename, local_tpl_file));
  boost::scoped_ptr<FabGeometryTemplate> tpl(
      FabGeometryTemplate::CheckCreateFromFile(local_tpl_file));
  //tpl->CheckInstantiate();
  //LOG(INFO) << "> Instantiated template - OK";

  std::ofstream samples_stream;
  LOG(INFO) << "> Opening samples stream: " << FLAGS_samples_out_file
            << (FLAGS_restart ? " (Append) " : " (Overwrite) ") << " - OK";
  samples_stream.open(
      FLAGS_samples_out_file.c_str(),
      (FLAGS_restart ?
       (std::fstream::out | std::fstream::app | std::fstream::ate) :
       (std::fstream::out | std::fstream::trunc)));

  std::ofstream geometry_stream;
  if (!FLAGS_geometry_index_out_file.empty()) {
    LOG(INFO) << "> Opening Geo stream: " << FLAGS_geometry_index_out_file
              << (FLAGS_restart ? " (Append) " : " (Overwrite) ") << " - OK";
    geometry_stream.open(
        FLAGS_geometry_index_out_file.c_str(),
        (FLAGS_restart ?
         (std::fstream::out | std::fstream::app | std::fstream::ate) :
         (std::fstream::out | std::fstream::trunc)));
  }
  LOG(INFO) << "Setting Geo dir to: " << FLAGS_geometry_out_dir;
  LOG(INFO) << "Setting Local Geo dir to: " << FLAGS_local_geometry_out_dir;
  SamplingMapper mapper(FLAGS_control_names,
                        tpl.get(),
                        samples_stream,
                        geometry_stream,
                        FLAGS_local_geometry_out_dir,
                        FLAGS_geometry_out_dir);
  if (FLAGS_restart) {
    LOG(INFO) << "> Re-initializing from previous run";
    mapper.SkipProcessedSamples(FLAGS_samples_out_file);
    mapper.InitializeGeometryCache(FLAGS_geometry_index_out_file);
    LOG(INFO) << "> ---> OK";
  }

  if (FLAGS_action == "ADAPT") {
    LOG(INFO) << "> Running ADAPTIVE sampling...";

    // Also write Deltas to filename
    std::ofstream deltas_stream;
    {
      const string deltas_filename = FLAGS_samples_out_file + ".DELTAS";
      LOG(INFO) << "> Opening deltas stream: " << deltas_filename
                << (FLAGS_restart ? " (Append) " : " (Overwrite) ") << " - OK";
      deltas_stream.open(
          deltas_filename.c_str(),
          (FLAGS_restart ?
           (std::fstream::out | std::fstream::app | std::fstream::ate) :
           (std::fstream::out | std::fstream::trunc)));
    }

    HybridLookupSpec lookup_spec;
    boost::scoped_ptr<HybridLookup> lookup(
        LoadHybridLookupFromFile(
            &lookup_spec, tpl.get(), &mapper.geo_cache(), &deltas_stream));
    // need to update lookup with computed properties
    mapper.SetLookup(lookup.get());

    vector<const Sample*> new_samples;
    lookup->GetSamplesWithoutProperties(&new_samples);  // If prev run failed
    LOG(INFO) << "> SAMPLES TO PROCESS: " << new_samples.size();
    for (const Sample* s : new_samples) {
      LOG(INFO) << "Processing sample: " << *s;
      mapper.ProcessSample(*s);
      LOG(INFO) << " --> Ok";
    }
    new_samples.clear();

    const int cuboids_end =
        (FLAGS_cuboids_to_process < 0 ?
         lookup->n_roots() :
         fmin(lookup->n_roots(), FLAGS_cuboid_id + FLAGS_cuboids_to_process));

    bool not_done = true;  // at least one new subdivision in previous step
    while (not_done) {
      not_done = false;

      for (int cube_id = FLAGS_cuboid_id; cube_id < cuboids_end; ++cube_id) {
        LOG(INFO) << "> PROCESSING ROOT CUBOID: " << cube_id;

        if (lookup->root(cube_id)->tree_depth() >= FLAGS_max_depth) {
          VLOG(1) << "  (Root cuboid already at max depth)";
          // But some leaves may still be high up in the tree, so
          // not a terminating condition
        }

        vector<std::pair<Hypercuboid*, int> > leaves;
        lookup->root(cube_id)->GetLeaves(&leaves);
        LOG(INFO) << "> Got " << leaves.size() << " leaves for root " << cube_id;

        for (const std::pair<Hypercuboid*, int>& pr : leaves) {
          Hypercuboid* leaf = pr.first;
          const int leaf_depth = pr.second;

          VLOG(1) << "> ... Processing leaf " << *leaf
                  << " at depth " << leaf_depth;
          if (leaf_depth >= FLAGS_max_depth) {
            VLOG(1) << "    .... SKIPPING -> max depth "
                    << FLAGS_max_depth << " reached";
            continue;
          } else {
            VLOG(1) << "    .... STARTING SUBDIVISION ....";
          }

          new_samples.clear();
          BackupHybridLookup(*lookup, &lookup_spec);
          vector<int> dims;
          mapper.GetSubdivisionDimensions(*leaf, lookup.get(), &dims);
          if (dims.empty()) {
            VLOG(1) << "       --> Subdivision Dimension --> NONE";
            continue;
          }

          // Possibly try more than one dimension
          bool subdivided = false;
          for (int d : dims) {
            VLOG(1) << "       --> Subdivision Dimension --> " << d
                    << " (" << lookup->GetDimension(d).name() << ")";

            subdivided = lookup->SubdivideOne(leaf, d, &new_samples);
            VLOG(1) << "Subdivision: " << (subdivided ? "Ok" : "Skipped");
            if (subdivided) {
              break;
            }
          }
          BackupHybridLookup(*lookup, &lookup_spec);

          if (subdivided) {
            not_done = true;
            mapper.ProcessSamples(new_samples);
            new_samples.clear();
            samples_stream.flush();
            VLOG(1) << "    .... SUBDIVISION DONE ....";
          } else {
            VLOG(1) << "    .... XXX SKIPPED SUBDIVISION ....";
          }
        }
      }
    }
  } else if (FLAGS_action == "UNI") {
    LOG(INFO) << "> Running on UNIFORM samples...";
    while (!std::cin.eof()) {
      char buffer[1000];
      std::cin.getline(buffer, 1000);

      string input(buffer);
      LOG(ERROR) << "> Setting controls to: " << input;
      if (input.empty()) continue;

      mapper.ProcessSample(input);
    }
  }
  LOG(INFO) << "Mapper finished";

  return 0;
}
