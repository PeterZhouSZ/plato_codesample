cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
enable_testing()

FIND_PACKAGE(Qt4 REQUIRED)
SET(QT_USE_QTOPENGL TRUE)
SET(QT_USE_QTXML TRUE)
INCLUDE(${QT_USE_FILE})
ADD_DEFINITIONS(${QT_DEFINITIONS})

find_package(GLUT REQUIRED)
find_package(OpenGL REQUIRED)

include_directories(${OPENGL_INCLUDE_DIR})
include_directories(${GLUT_INCLUDE_DIR})

# Because some headers in this subdir are relative to this subdir
# and we don't want to change this, b/c it may break
# the xcode project that also builds this binary and is useful for
# debugging.
include_directories(".")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BUILD_DIR}/rootrepo/fab/qt_templatizer)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${BUILD_DIR}/rootrepo/fab/qt_templatizer)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${BUILD_DIR}/rootrepo/fab/qt_templatizer)

PROJECT(fab_qt_templatizer)

SET(fab_qt_templatizer_SOURCES
  color_scheme.cc
  graphical_editors_qt.cc
  main.cpp
  template_model_qt.cc
  template_viewer_qt.cc
  viewer_main_window_qt.cc
  widgets/test_customizer_dialog_qt.cc
  widgets/constraints_dialog_qt.cc
  widgets/metavar_dialog_qt.cc
  widgets/mutable_dialog_qt.cc)

SET(fab_qt_templatizer_HEADERS
  color_scheme.h
  graphical_editors_qt.h
  template_model_qt.h
  template_viewer_qt.h
  viewer_main_window_qt.h
  widgets/test_customizer_dialog_qt.h
  widgets/constraints_dialog_qt.h
  widgets/metavar_dialog_qt.h
  widgets/mutable_dialog_qt.h
  ui/ui_gen-inl.h)

SET(fab_qt_templatizer_FORMS
  designer_ui/test_customizer_dialog.ui
  designer_ui/constraints_dialog.ui
  designer_ui/metavar_dialog.ui
  designer_ui/mutable_dialog.ui)

SET(fab_qt_templatizer_RESOURCES
  ui_resources.qrc)

QT4_WRAP_CPP(fab_qt_templatizer_HEADERS_MOC ${fab_qt_templatizer_HEADERS})
QT4_WRAP_UI(fab_qt_templatizer_FORMS_HEADERS ${fab_qt_templatizer_FORMS})
QT4_ADD_RESOURCES(fab_qt_templatizer_RESOURCES_RCC ${fab_qt_templatizer_RESOURCES})

MESSAGE("FORMS: " ${fab_qt_templatizer_FORMS_HEADERS})

ADD_EXECUTABLE(PlatoEng
  ${fab_qt_templatizer_SOURCES}
  ${fab_qt_templatizer_HEADERS_MOC}
  ${fab_qt_templatizer_FORMS_HEADERS}
  ${fab_qt_templatizer_RESOURCES_RCC})
TARGET_LINK_LIBRARIES(PlatoEng
  fab_geometry_template_precomp_util
  ${FORCE_LINK_ON}
  fab_geometry_template_lib
  ${FORCE_LINK_OFF}
  ${FORCE_LINK_ON}
  fab_geometry_template_openscad_lib
  ${FORCE_LINK_OFF}
  fab_geometry_template_customizer_lib
  gecodesearch
  gflags
  QGLViewer
  ${GLUT_LIBRARIES}
  ${OPENGL_LIBRARIES}
  ${QT_LIBRARIES})
