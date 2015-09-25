cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
enable_testing()

include(FindProtobuf)
find_package(Protobuf REQUIRED)
find_package(GLUT REQUIRED)
find_package(OpenGL REQUIRED)

include_directories(${OPENGL_INCLUDE_DIR})
include_directories(${GLUT_INCLUDE_DIR})
include_directories(${PROTOBUF_INCLUDE_DIRS})

FIND_PACKAGE(Qt4)
SET(QT_USE_QTXML 1)
INCLUDE(${QT_USE_FILE})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BUILD_DIR}/rootrepo/fab/geometry/template)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${BUILD_DIR}/rootrepo/fab/geometry/template)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${BUILD_DIR}/rootrepo/fab/geometry/template)

PROJECT(fab_geometry_template)

ROOTREPO_PROTOBUF_GENERATE_CPP(
  fab/geometry/template ProtoSources ProtoHeaders
  constraints.proto
  operations.proto
  template.proto)

ADD_LIBRARY(fab_geometry_template_lib STATIC
  # TODO: Because both google protocol buffer library and the fab template
  # library employ registration mechanism via global variable initialization,
  # there are issues with static initialization order. These need to be
  # fixed e.g. via never calling protocol buffer methods that rely on the
  # protocol buffer Reflection to be initialized from the fab template registration
  # rutines.
  # Note: currently we just hope that the initialization order is correct
  # across translation units.
  ${ProtoSources}
  ${ProtoHeaders}
  bounds.cc
  bounds.h
  checks.cc
  checks.h
  constraints/constraints.cc
  constraints/constraints.h
  constraints/expression.h
  constraints/expression.cc
  constraints/model.h
  constraints/model.cc
  constraints/model_helper.h
  constraints/model_helper.cc
  curves.cc
  curves.h
  editor.cc
  editor.h
  examples.cc
  examples.h
  interpreter.cc
  interpreter.h
  library.cc
  library.h
  operations.cc
  operations.h
  operations/combine_operations.cc
  operations/combine_operations.h
  operations/create_operations.cc
  operations/create_operations.h
  operations/modify_operations.cc
  operations/modify_operations.h
  operations/special_operations.cc
  operations/special_operations.h
  params.cc
  params.h
  proto_convert.h
  search_paths.cc
  search_paths.h
  shape.cc
  shape.h
  template.cc
  template.h
  template_params.cc
  template_params.h)
TARGET_LINK_LIBRARIES(fab_geometry_template_lib
  fab_geometry_lib
  util_lib
  boost_filesystem
  boost_regex
  boost_system
  boost_timer
  boost_thread-mt
  carve
  gecodeminimodel
  gecodefloat
  gecodeint
  gecodekernel
  gecodesearch
  gecodesupport
  gflags
  glog
  OpenMeshCore
  OpenMeshTools
  openvdb
  tbb
  Half
  ${GLUT_LIBRARIES}
  ${OPENGL_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
  ${QT_LIBRARIES}
  ${TRIANGLE_LIBRARIES})

ADD_LIBRARY(fab_geometry_template_openscad_lib STATIC
  operations/openscad_operations.h
  operations/openscad_operations.cc)
TARGET_LINK_LIBRARIES(fab_geometry_template_openscad_lib
  fab_geometry_template_lib
  boost_filesystem
  CGAL
  opencsg
  gmp
  mpfr
  openscadlib)

ADD_EXECUTABLE(simple_template_main simple_main.cc)
TARGET_LINK_LIBRARIES(simple_template_main
  gflags
  glog
  ${FORCE_LINK_ON}
  fab_geometry_template_lib
  ${FORCE_LINK_OFF}
  ${FORCE_LINK_ON}
  fab_geometry_template_openscad_lib
  ${FORCE_LINK_OFF})

ADD_SUBDIRECTORY(precomp)

ADD_LIBRARY(fab_geometry_template_customizer_lib STATIC
  customizer.cc
  customizer.h
  smart_customizer.cc
  smart_customizer.h)
TARGET_LINK_LIBRARIES(fab_geometry_template_customizer_lib
  ${FORCE_LINK_ON}
  fab_geometry_template_lib
  ${FORCE_LINK_OFF}
  fab_geometry_template_precomp_util)

# TESTS ------------------------------------------------------------------------

ADD_EXECUTABLE(curves_test_bin curves_test.cc)
TARGET_LINK_LIBRARIES(curves_test_bin
  gtest
  gtest_main
  ${FORCE_LINK_ON}
  fab_geometry_template_lib
  ${FORCE_LINK_OFF})
ADD_TEST(curves_test
         ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/curves_test_bin
         --logtostderr)

ADD_EXECUTABLE(create_operations_test_bin operations/create_operations_test.cc)
TARGET_LINK_LIBRARIES(create_operations_test_bin
  gflags
  gtest
  ${FORCE_LINK_ON}
  fab_geometry_template_lib
  ${FORCE_LINK_OFF})
ADD_TEST(create_operations_test
         ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/create_operations_test_bin
         --logtostderr)

ADD_EXECUTABLE(customizer_test_bin customizer_test.cc)
TARGET_LINK_LIBRARIES(customizer_test_bin
  gflags
  glog
  gtest
  fab_geometry_template_customizer_lib
  ${FORCE_LINK_ON}
  fab_geometry_template_lib
  ${FORCE_LINK_OFF})
ADD_TEST(customizer_test
         ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/customizer_test_bin
         --logtostderr)

ADD_EXECUTABLE(openscad_operations_test_bin operations/openscad_operations_test.cc)
TARGET_LINK_LIBRARIES(openscad_operations_test_bin
  gflags
  glog
  gtest
  ${FORCE_LINK_ON}
  fab_geometry_template_openscad_lib
  ${FORCE_LINK_OFF})
ADD_TEST(openscad_operations_test
         ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/openscad_operations_test_bin
         --logtostderr)

ADD_EXECUTABLE(checks_test_bin checks_test.cc)
TARGET_LINK_LIBRARIES(checks_test_bin
  gflags
  glog
  gtest
  ${FORCE_LINK_ON}
  fab_geometry_template_lib
  ${FORCE_LINK_OFF})
ADD_TEST(checks_test
         ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/checks_test_bin
         --logtostderr)

ADD_EXECUTABLE(expression_test_bin constraints/expression_test.cc)
TARGET_LINK_LIBRARIES(expression_test_bin
  gflags
  glog
  gtest
  ${FORCE_LINK_ON}
  fab_geometry_template_lib
  ${FORCE_LINK_OFF})
ADD_TEST(expression_test
         ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/expression_test_bin
         --logtostderr)

ADD_EXECUTABLE(constraints_test_bin constraints/constraints_test.cc)
TARGET_LINK_LIBRARIES(constraints_test_bin
  gflags
  glog
  gtest
  ${FORCE_LINK_ON}
  fab_geometry_template_lib
  ${FORCE_LINK_OFF})
ADD_TEST(constraints_test
         ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/constraints_test_bin
         --logtostderr)

ADD_EXECUTABLE(model_test_bin constraints/model_test.cc)
TARGET_LINK_LIBRARIES(model_test_bin
  gecodesearch
  gflags
  glog
  gtest
  ${FORCE_LINK_ON}
  fab_geometry_template_lib
  ${FORCE_LINK_OFF})
ADD_TEST(model_test
         ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/constraints_test_bin
         --logtostderr)
