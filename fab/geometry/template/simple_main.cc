#include <iostream>
using std::cin;
using std::cout;
using std::endl;

#include <boost/smart_ptr/shared_ptr.hpp>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <google/protobuf/text_format.h>

#include "fab/geometry/template/interpreter.h"
#include "fab/geometry/template/template.h"
#include "fab/geometry/template/library.h"
#include "fab/geometry/template/template.pb.h"
#include "util/proto_util.h"

DEFINE_string(tpl_filename,
              "/Users/shumash/Documents/3DModels/templates/glass.ascii_proto",
              "Template filename to load; must be FabTemplateProto message "
              "in protocol buffer ascii or binary format; "
              "if filename is empty, will start with an empty template.");

using mit_plato::FabGeometryTemplate;
using mit_plato::FabTemplateProto;
using mit_plato::PlatoInterpreter;


PlatoInterpreter* InitializeInterpreter() {
  if (FLAGS_tpl_filename.empty()) {
    cout << "--> Starting with a blank template" << endl;
    return new PlatoInterpreter(boost::shared_ptr<FabGeometryTemplate>(new FabGeometryTemplate()));
  } else {
    cout << "--> Loading template from file: " << FLAGS_tpl_filename;
    FabTemplateProto tpl_proto;
    CHECK(mds_util::ReadProtoFromFile(FLAGS_tpl_filename, &tpl_proto));
    boost::shared_ptr<FabGeometryTemplate> tpl(CHECK_NOTNULL(
       FabGeometryTemplate::FromSpec(tpl_proto)));
    CHECK(tpl->Instantiate());
    CHECK_GT(tpl->shapes().size(), 0);
    cout << " --> Successfully instantiated!" << endl;
    return new PlatoInterpreter(tpl, FLAGS_tpl_filename);
  }
}

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  FLAGS_logtostderr = true;

  if (!mit_plato::FabTemplateLibrary::InitLibrary()) {
    LOG(ERROR) << "Failed to init library!";
  }

  boost::scoped_ptr<PlatoInterpreter> interpreter(InitializeInterpreter());

  string command;
  while (true) {
    cout << "Commands: " << endl;
    interpreter->CoutCommands();
    cout << " > Enter command: ";
    std::getline(cin, command);

    if (interpreter->IsQuitCmd(command)) {
      break;
    }

    cout << endl;
    if (!interpreter->ExecuteTopLevelCmd(command)) {
      cout << "   -->ERROR: unknown command \"" << command << "\"" << endl << endl;
    }
    cout << endl;
  }

  cout << endl;
  cout << "--> Preparing to quit..." << endl;
  cout << "  > Save template before quitting [y/n]? ";
  if (interpreter->GetYesNoAnswerCmd()) {
    interpreter->SaveTemplateCmd();
  }

  return 0;
}
