cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

#include(FindProtobuf)
#find_package(Protobuf REQUIRED)

include_directories(${PROTOBUF_INCLUDE_DIRS})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BUILD_DIR}/rootrepo/fab/plato_web2)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${BUILD_DIR}/rootrepo/fab/plato_web2)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${BUILD_DIR}/rootrepo/fab/plato_web2)

PROJECT(fab_plato_web2)


ADD_LIBRARY(customizer2_lib
  customizer2.cc
  mesh_encoder.cc)

ADD_EXECUTABLE(plato_web2
  server.cc
  main.cc)
TARGET_LINK_LIBRARIES(plato_web2
  customizer2_lib
  ${FORCE_LINK_ON}
  fab_geometry_template_lib
  ${FORCE_LINK_OFF}
  ${FORCE_LINK_ON}
  fab_geometry_template_openscad_lib
  ${FORCE_LINK_OFF}
  fab_geometry_template_customizer_lib
  util_lib
  boost_system
  boost_thread
  boost_signals
  boost_program_options
  boost_date_time
  boost_random
  gflags
  glog
#  wt
#  wthttp
#  modpbase64
  pthread)


ADD_EXECUTABLE(plato_web2_debug
  mesh_encoder.cc
  ws_server.cc)
TARGET_LINK_LIBRARIES(plato_web2_debug
  ${FORCE_LINK_ON}
  fab_geometry_template_lib
  ${FORCE_LINK_OFF}
  ${FORCE_LINK_ON}
  fab_geometry_template_openscad_lib
  ${FORCE_LINK_OFF}
  fab_geometry_template_customizer_lib
  util_lib
  boost_system
  boost_thread
  boost_signals
  boost_program_options
  boost_date_time
  boost_random
  gflags
  glog
  wt
  wthttp
  modpbase64
  pthread)
