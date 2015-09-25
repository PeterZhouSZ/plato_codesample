#include "viewer_main_window_qt.h"

#include <utility>
#include <set>
#include <GL/glut.h>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/filesystem.hpp>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <google/protobuf/text_format.h>
#include <qapplication.h>

#include "fab/geometry/tri_mesh.h"
#include "fab/geometry/template/examples.h"
#include "fab/geometry/template/library.h"
#include "fab/geometry/template/template.h"
#include "fab/geometry/template/template.pb.h"

#include "fab/geometry/template/customizer.h"
#include "fab/geometry/template/create_operations.h"
#include "util/proto_util.h"

DEFINE_string(tpl_filename,
              "",
              "Template filename to load.");

DECLARE_bool(logtostderr);
DECLARE_int32(v);
DECLARE_bool(enable_analytics);

using mit_plato::FabGeometryTemplate;
using mit_plato::FabTemplateProto;
using mit_plato::OperationNode;

#include "fab/geometry/algorithms2d.h"
using mds::algo_2d::PolygonPtAligner;
using mds::algo_2d::Indexer;
using Eigen::Vector3d;


class NotifyingApplication : public QApplication {
 public:
  NotifyingApplication(int& argc, char** argv )
      : QApplication(argc, argv) {}

  virtual bool notify(QObject* receiver, QEvent* event) {
    try {
      return QApplication::notify( receiver, event );
    } catch (std::exception& e ) {
      LOG(ERROR) << "Exception caught: " << e.what();
      return false;
    } catch (...) {
      LOG(ERROR) << "Caught some strange exception";
      return false;
    }
  }
};

void DummyCallback(mit_plato::TemplateCustomizer::Status status,
                   mit_plato::TemplateCustomizer* c) {
  LOG(INFO) << "Geometry update done.";
}

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  FLAGS_logtostderr = true;

  if (!mit_plato::FabTemplateLibrary::InitLibrary()) {
    LOG(ERROR) << "Failed to init library!";
  }

  glutInit(&argc, argv);

  NotifyingApplication* app = new NotifyingApplication(argc, argv);

  QMainWindow window;
  mds::ViewerMainWindow ui;
  ui.setupUi(&window);

  boost::shared_ptr<mit_plato::FabGeometryTemplate> tpl;
  if (!FLAGS_tpl_filename.empty()) {
    tpl.reset(mit_plato::FabGeometryTemplate::CheckCreateFromFile(FLAGS_tpl_filename));
    CHECK(tpl->Instantiate());
    CHECK_GT(tpl->shapes().size(), 0);
    LOG(INFO) << "Instantiated template";
  } else {
    tpl.reset(new FabGeometryTemplate());
  }
  LOG(INFO) << "Created template";

  if (FLAGS_enable_analytics) {
    set<std::pair<double, std::pair<const OperationNode*, string> > > node_times;
    for (OperationNode* n : tpl->nodes()) {
      const map<string, double>& keyed_times = n->run_times();
      for (const auto& p : keyed_times) {
        node_times.insert(std::make_pair(p.second, std::make_pair(n, p.first)));
      }
    }
    for (const auto& p : node_times) {
      LOG(INFO) << "Run time: " << p.first << " - " << p.second.first->name()
                << " - Key: '" << p.second.second << "'";
    }
  }
  ui.setTemplate(tpl, FLAGS_tpl_filename);

  window.resize(1200, 800);
  app->setActiveWindow(&window);
  window.show();

  // Run main loop
  int ret = app->exec();
  delete app;
  return ret;
}
