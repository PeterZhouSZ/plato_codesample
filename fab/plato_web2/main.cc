#include <glog/logging.h>
#include <gflags/gflags.h>

#include "fab/geometry/template/library.h"
#include "fab/plato_web2/server.h"


DEFINE_int32(port, 8080, "Port to use for connection.");
DEFINE_string(docroot, "", "Root of docs served through http.");


int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  mit_plato::web::Customizer2Factory::Init();

  LOG(INFO) << "Setting up.";
  mit_plato::web::PlatoServer server;
  server.Init(FLAGS_port, FLAGS_docroot);
  server.Run();

  // TODO: handle Control-C appropriately

  return 0;
}
