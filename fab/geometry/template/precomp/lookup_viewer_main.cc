#include <qapplication.h>
#include <QKeyEvent>
#include <QEvent>

#include <string>
using std::string;
#include <vector>
using std::vector;

#include <iostream>
using std::cout;
using std::cin;
using std::endl;

#include <QGLViewer/qglviewer.h>
#include <boost/smart_ptr/scoped_ptr.hpp>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include "fab/geometry/template/precomp/lookup.h"
#include "fab/geometry/template/precomp/lookup_viewer.h"
#include "fab/geometry/template/customizer.h"
#include "fab/geometry/template/library.h"
#include "fab/geometry/template/template.pb.h"
#include "util/proto_util.h"

DEFINE_string(customizer_spec, "",
              "Location of the appropriate CustomizerSpec proto.");

using namespace mit_plato;

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (!mit_plato::FabTemplateLibrary::InitLibrary()) {
    LOG(ERROR) << "Failed to init library!";
  }

  // Read command lines arguments.
  QApplication application(argc,argv);

  CustomizerSpec spec;
  CHECK(mds_util::ReadProtoFromFile(FLAGS_customizer_spec, &spec))
      << "Failed to parse CustomizerSpec from file " << FLAGS_customizer_spec;
  boost::scoped_ptr<TemplateCustomizer> customizer(
      CustomizerFactory::FromSpec(spec));

  // Instantiate the viewer.
  LookupViewer viewer(*customizer);
  viewer.setWindowTitle("lookup visualization");
  application.installEventFilter(&viewer);

  // Make the viewer window visible on screen.
  viewer.print_info();
  viewer.show();

  // Run main loop.
  return application.exec();
}
