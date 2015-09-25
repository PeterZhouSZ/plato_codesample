#ifndef _FAB_PLATO_WEB2_SERVER_H__
#define _FAB_PLATO_WEB2_SERVER_H__

#include <map>
#include <string>
using std::string;

#include <websocketpp/server.hpp>

#include "fab/plato_web2/customizer2.h"
#include "fab/plato_web2/wspp_config.h"


namespace mit_plato {
namespace web {

namespace ws = websocketpp;
typedef ws::server<wspp_server_config> server;

// Sends the following formats:
//
// 1. JSON text for design info
// { seeds: [ "1.5_6", "2.0_3" ],
//   controls: [
//     { name: "width",
//       range: [ 1, 20 ],
//       type: "INT" | "DOUBLE" | "BOOL",
//       value: 3.5
//      }, ... ],
//   name: "Template Name"
// }
//
// 2. JSON text for changed bounds
// { bounds:
//   [{ name: "control_name",
//      intervals: [ [start1, end1], [start2, end2] ] }, ...]
// }
//
// 3. binary for geometry (see mesh_encoder.h)
//
// 4. { grid: { ... see GridEncoder } }
class PlatoServer {
 public:
  PlatoServer() {}
  virtual ~PlatoServer();

  void Init(int port, const string& docroot);
  void Run();

 protected:
  void on_open(ws::connection_hdl hdl);
  void on_close(ws::connection_hdl hdl);

  void on_http(ws::connection_hdl hdl);
  void on_message(ws::connection_hdl hdl,
                  server::message_ptr msg);

  void SendGeometryResponse(ws::connection_hdl hdl,
                            const string& binary_geometry_data);

 private:
  server server_;
  string docroot_;

  std::map<ws::connection_hdl, TemplateCustomizer2*> customizers_;
  std::map<string, string> customizer_specs_;
};


class ServerUtils {
 public:
  static string mime_type(const string& file);

  static string get_extension(const string& path);

  static bool set_up_file_response(const string& full_path,
                                   server::connection_ptr con);

  static string name_from_resource(const string& uri_resource);
};

}  // namespace web
}  // namespace mit_plato

#endif  // _FAB_PLATO_WEB2_SERVER_H__
