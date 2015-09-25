cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
enable_testing()

include(FindProtobuf)
find_package(Protobuf REQUIRED)
find_package(GLUT REQUIRED)
find_package(OpenGL REQUIRED)
FIND_PACKAGE(Qt4 REQUIRED)
SET(QT_USE_QTOPENGL TRUE)

include_directories(${OPENGL_INCLUDE_DIR})
include_directories(${GLUT_INCLUDE_DIR})
include_directories(${PROTOBUF_INCLUDE_DIRS})

FIND_PACKAGE(Qt4)
SET(QT_USE_QTXML 1)
INCLUDE(${QT_USE_FILE})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY
  ${BUILD_DIR}/rootrepo/fab/geometry/template/precomp)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY
  ${BUILD_DIR}/rootrepo/fab/geometry/template/precomp)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY
  ${BUILD_DIR}/rootrepo/fab/geometry/template/precomp)

PROJECT(fab_geometry_template_precomp)

ADD_LIBRARY(fab_geometry_template_precomp_util STATIC
  aws_util.cc
  aws_util.h
  key_util.cc
  key_util.h
  geometry_cache.cc
  geometry_cache.h
  lookup.cc
  lookup.h
  stats.h
  stats.cc
  sampling_mapper.cc
  sampling_mapper.h)
TARGET_LINK_LIBRARIES(fab_geometry_template_precomp_util
  gflags
  glog
  ${FORCE_LINK_ON}
  fab_geometry_template_lib
  ${FORCE_LINK_OFF}
  ${FORCE_LINK_ON}
  fab_geometry_template_openscad_lib
  ${FORCE_LINK_OFF})

ADD_LIBRARY(fab_geometry_template_precomp_viewer_lib
  lookup_viewer.h
  lookup_viewer.cc
  lookup_renderer.h
  lookup_renderer.cc)
TARGET_LINK_LIBRARIES(fab_geometry_template_precomp_viewer_lib
  gflags
  glog
  fab_geometry_template_customizer_lib
  fab_geometry_template_precomp_util
  ${FORCE_LINK_ON}
  fab_geometry_template_lib
  ${FORCE_LINK_OFF}
  ${FORCE_LINK_ON}
  fab_geometry_template_openscad_lib
  ${FORCE_LINK_OFF}
  QGLViewer
  ${GLUT_LIBRARIES}
  ${OPENGL_LIBRARIES}
  ${QT_LIBRARIES})


# EXECUTABLES ------------------------------------------------------------------

ADD_EXECUTABLE(precomp_test_main test_main.cc)
TARGET_LINK_LIBRARIES(precomp_test_main
  fab_geometry_template_precomp_util)

ADD_EXECUTABLE(prune_cache_main prune_cache_main.cc)
TARGET_LINK_LIBRARIES(prune_cache_main
  ${FORCE_LINK_ON}
  fab_geometry_template_lib
  ${FORCE_LINK_OFF}
  ${FORCE_LINK_ON}
  fab_geometry_template_openscad_lib
  ${FORCE_LINK_OFF}
  fab_geometry_template_precomp_util)

ADD_EXECUTABLE(optimize_cache_main optimize_cache_main.cc)
TARGET_LINK_LIBRARIES(optimize_cache_main
  ${FORCE_LINK_ON}
  fab_geometry_template_lib
  ${FORCE_LINK_OFF}
  ${FORCE_LINK_ON}
  fab_geometry_template_openscad_lib
  ${FORCE_LINK_OFF}
  fab_geometry_template_precomp_util)

ADD_EXECUTABLE(compute_missing_delta_geo_main compute_missing_delta_geo.cc)
TARGET_LINK_LIBRARIES(compute_missing_delta_geo_main
  fab_geometry_template_precomp_util
  ${FORCE_LINK_ON}
  fab_geometry_template_lib
  ${FORCE_LINK_OFF}
  ${FORCE_LINK_ON}
  fab_geometry_template_openscad_lib
  ${FORCE_LINK_OFF})

ADD_EXECUTABLE(missing_delta_geo_main missing_delta_geo.cc)
TARGET_LINK_LIBRARIES(missing_delta_geo_main
  fab_geometry_template_precomp_util
  ${FORCE_LINK_ON}
  fab_geometry_template_lib
  ${FORCE_LINK_OFF}
  ${FORCE_LINK_ON}
  fab_geometry_template_openscad_lib
  ${FORCE_LINK_OFF})

ADD_EXECUTABLE(hadoop_mapper_main hadoop_mapper_main.cc)
TARGET_LINK_LIBRARIES(hadoop_mapper_main
  boost_filesystem
  fab_geometry_template_precomp_util
  ${FORCE_LINK_ON}
  fab_geometry_template_lib
  ${FORCE_LINK_OFF}
  ${FORCE_LINK_ON}
  fab_geometry_template_openscad_lib
  ${FORCE_LINK_OFF})

ADD_EXECUTABLE(aggregate_hybrid_main aggregate_hybrid_main.cc)
TARGET_LINK_LIBRARIES(aggregate_hybrid_main
  fab_geometry_template_precomp_util
  ${FORCE_LINK_ON}
  fab_geometry_template_lib
  ${FORCE_LINK_OFF}
  ${FORCE_LINK_ON}
  fab_geometry_template_openscad_lib
  ${FORCE_LINK_OFF})

ADD_EXECUTABLE(precomp_main precomp_main.cc)
TARGET_LINK_LIBRARIES(precomp_main
  gecodesearch
  gflags
  glog
  fab_geometry_template_precomp_util
  ${FORCE_LINK_ON}
  fab_geometry_template_lib
  ${FORCE_LINK_OFF}
  ${FORCE_LINK_ON}
  fab_geometry_template_openscad_lib
  ${FORCE_LINK_OFF})

ADD_EXECUTABLE(precomp_lookup_viewer_main
  lookup_viewer_main.cc)
TARGET_LINK_LIBRARIES(precomp_lookup_viewer_main
  gflags
  glog
  fab_geometry_template_precomp_viewer_lib
  fab_geometry_template_customizer_lib
  fab_geometry_template_precomp_util
  ${FORCE_LINK_ON}
  fab_geometry_template_lib
  ${FORCE_LINK_OFF}
  ${FORCE_LINK_ON}
  fab_geometry_template_openscad_lib
  ${FORCE_LINK_OFF}
  QGLViewer
  ${GLUT_LIBRARIES}
  ${OPENGL_LIBRARIES}
  ${QT_LIBRARIES})

ADD_EXECUTABLE(rm_dupes_main
  rm_dupes_main.cc)
TARGET_LINK_LIBRARIES(rm_dupes_main
  gflags
  glog
  fab_geometry_template_lib
  util_lib)

ADD_EXECUTABLE(timing_stats_main
  timing_stats_main.cc)
TARGET_LINK_LIBRARIES(timing_stats_main
  customizer2_lib
  gflags
  glog
  fab_geometry_template_customizer_lib
  fab_geometry_template_precomp_util
  ${FORCE_LINK_ON}
  fab_geometry_template_lib
  ${FORCE_LINK_OFF}
  ${FORCE_LINK_ON}
  fab_geometry_template_openscad_lib
  ${FORCE_LINK_OFF}
  util_lib)
