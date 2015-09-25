#ifndef _FAB_PLATO_WEB2_WSPP_CONFIG_H__
#define _FAB_PLATO_WEB2_WSPP_CONFIG_H__

#include <websocketpp/config/asio_no_tls.hpp>

namespace ws = websocketpp;

namespace mit_plato {
namespace web {

// Configuration used for our version of the websocketpp server;
// most settings are default (copied from config::asio), but e.g.
// log levels are set differently.
struct wspp_server_config : public ws::config::asio {
  typedef wspp_server_config type;
  typedef ws::config::asio base;

  typedef base::concurrency_type concurrency_type;

  typedef base::request_type request_type;
  typedef base::response_type response_type;

  typedef base::message_type message_type;
  typedef base::con_msg_manager_type con_msg_manager_type;
  typedef base::endpoint_msg_manager_type endpoint_msg_manager_type;

  typedef base::alog_type alog_type;
  typedef base::elog_type elog_type;

  typedef base::rng_type rng_type;

  struct transport_config : public base::transport_config {
    typedef base::concurrency_type concurrency_type;
    typedef base::elog_type elog_type;
    typedef base::alog_type alog_type;
    typedef base::request_type request_type;
    typedef base::response_type response_type;
    // static bool const enable_multithreading = true;
  };

  typedef ws::transport::asio::endpoint<transport_config>
  transport_type;

  static const ws::log::level elog_level =
      ws::log::elevel::none;
  static const ws::log::level alog_level =
      ws::log::alevel::none;
};


}  // namespace web
}  // namespace mit_plato

#endif  // _FAB_PLATO_WEB2_WSPP_CONFIG_H__
