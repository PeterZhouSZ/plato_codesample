cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(Qt4 REQUIRED)
set(QT_USE_QTOPENGL TRUE)

include(FindProtobuf)
find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIRS})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BUILD_DIR}/rootrepo/fab/geometry)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${BUILD_DIR}/rootrepo/fab/geometry)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${BUILD_DIR}/rootrepo/fab/geometry)

set(QT_USE_QTXML 1)
include(${QT_USE_FILE})

PROJECT(fab_geometry)

ROOTREPO_PROTOBUF_GENERATE_CPP(
  fab/geometry GeoProtoSources GeoProtoHeaders
  skeleton.proto)

ADD_LIBRARY(fab_geometry_lib STATIC
  ${GeoProtoSources}
  ${GeoProtoHeaders}
  convert.h
  algorithms2d.cc
  algorithms2d.h
  intersect.h
  intersect.cc
  transform_utils.h
  tri_mesh.cc
  tri_mesh.h
  mesh_renderer.h
  mesh_renderer.cc
  mesh_deformer.h
  mesh_deformer.cc
  skeleton.h
  skeleton.cc
  triangle_wrap.cc
  triangle_wrap.h)
TARGET_LINK_LIBRARIES(fab_geometry_lib
  util_lib
  gflags
  glog
  polyclipping
  newmat
  OpenMeshCore
  openvdb
  tbb
  Half
  pthread
  QGLViewer
  ${PROTOBUF_LIBRARIES}
  ${QT_LIBRARIES}
  # TODO: link all the necessary libs!!
  ${TRIANGLE_LIBRARIES})

ADD_SUBDIRECTORY(template)

# TESTS ------------------------------------------------------------------------

ADD_EXECUTABLE(algorithms2d_test_bin algorithms2d_test.cc)
TARGET_LINK_LIBRARIES(algorithms2d_test_bin
  boost_regex
  gflags
  glog
  gtest
  fab_geometry_lib)
ADD_TEST(algorithms2d_test
         ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/algorithms2d_test_bin
         --logtostderr)
