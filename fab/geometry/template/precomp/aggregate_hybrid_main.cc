#include <sstream>

#include <boost/smart_ptr/scoped_ptr.hpp>
#include <gflags/gflags.h>
#include <glog/logging.h>

#include "fab/geometry/template/precomp/lookup.h"
#include "fab/geometry/template/template.h"
#include "fab/geometry/template/library.h"
#include "fab/geometry/template/params.h"
#include "fab/geometry/template/template.pb.h"
#include "util/proto_util.h"

DEFINE_string(tpl_filename,
              "",
              "Template filename to load; must be FabTemplateProto message "
              "in protocol buffer ascii or binary format.");

DEFINE_int32(num_trees, 20,
             "The number of KD trees to aggregate into a hybrid lookup table "
             "will look for files of the form ${hybrid_lookup_spec_prefix}$i "
             "with i = 0...num_trees-1");

DEFINE_string(hybrid_lookup_spec_prefix, "",
              "File prefix of the specs of the format HybridLookupSpec.");

DEFINE_string(full_hybrid_lookup_spec, "",
              "Is read as the placeholder hybrid spec and populated with "
              "aggregated cuboids; must contain file with the full set "
              "of adaptive samples.");

using namespace mit_plato;

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  FLAGS_logtostderr = true;

  LOG(INFO) << "Begin";
  LOG(ERROR) << "Begin";

  CHECK(!FLAGS_tpl_filename.empty());
  CHECK(!FLAGS_hybrid_lookup_spec_prefix.empty());
  CHECK(!FLAGS_full_hybrid_lookup_spec.empty());
  CHECK_GT(FLAGS_num_trees, 0);

  if (!mit_plato::FabTemplateLibrary::InitLibrary()) {
    LOG(ERROR) << "Failed to init library!";
  }

  LOG(INFO) << "Reading template";
  boost::scoped_ptr<FabGeometryTemplate> tpl(
      FabGeometryTemplate::CheckCreateFromFile(FLAGS_tpl_filename));

  HybridLookupSpec result_spec;
  CHECK(mds_util::ReadProtoFromFile(
      FLAGS_full_hybrid_lookup_spec, &result_spec))
      << "Failed to parse lookup spec from file "
      << FLAGS_full_hybrid_lookup_spec;

  boost::scoped_ptr<HybridLookup> lookup(
      new HybridLookup(result_spec, tpl.get()));
  LOG(INFO) << "Created Hybrid Lookup.";

  LOG(INFO) << "Reading all the specs";
  for (int i = 0; i < FLAGS_num_trees; ++i) {
    stringstream ss;
    ss << FLAGS_hybrid_lookup_spec_prefix << i;

    HybridLookupSpec lookup_spec;
    CHECK(mds_util::ReadProtoFromFile(ss.str(), &lookup_spec))
        << "Failed to parse lookup spec from file " << ss.str();

    lookup_spec.clear_adaptive_samples_file();
    lookup_spec.mutable_grid_spec()->set_samples_file(
        result_spec.grid_spec().samples_file());
    LOG(INFO) << "Read spec " << i;

    lookup->AddCuboids(lookup_spec);
    lookup->AddGeoDeltas(lookup_spec.geo_deltas());
  }
  LOG(INFO) << "Read all " << FLAGS_num_trees << " specs";

  LOG(INFO) << "Writing spec to file";
  result_spec.clear_cuboid();
  lookup->CuboidsToSpec(&result_spec);
  lookup->GeoDeltasToSpec(result_spec.mutable_geo_deltas());
  mds_util::WriteASCIIProtoToFile(FLAGS_full_hybrid_lookup_spec, result_spec);
  LOG(INFO) << "Success";

  return 0;
}
