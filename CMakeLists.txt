cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
enable_testing()

set(QT_QMAKE_EXECUTABLE /usr/local/bin/qmake)
SET(CMAKE_VERBOSE_MAKEFILE on)
set(BUILD_DIR $ENV{BUILD_DIR})

message("Build Dir: " BUILD_DIR)

set(ROOT_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR})

# TODO: fix C++11 issues in a cross-compiler manner
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-c++11-extensions")

# TODO: This is a hack!! Fix it.
SET(TRIANGLE_LIBRARIES $ENV{TRIANGLE_LIBRARIES})
SET_SOURCE_FILES_PROPERTIES(${TRIANGLE_LIBRARIES}
  PROPERTIES
  EXTERNAL_OBJECT true
  GENERATED false)

# Sets two variables FORCE_LINK_ON and FORCE_LINK_OFF
# that should be used to force all symbols from a static library
# to be linked (useful when C++ implements a reflection mechanism,
# where all the used functions are not necessarily referenced
# in code.)
if(UNIX)
  # LINUX
  if("${CMAKE_SYSTEM}" MATCHES Linux)
    # Note: frounding-math not tested everywhere
    set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -frounding-math -g -ftest-coverage -fprofile-arcs -lpthread")
    set(FORCE_LINK_ON -Wl,-whole-archive)
    set(FORCE_LINK_OFF -Wl,-no-whole-archive)
  endif("${CMAKE_SYSTEM}" MATCHES Linux)
  # SGI IRIX
  if("${CMAKE_SYSTEM}" MATCHES IRIX)
    set(FORCE_LINK_ON -Wl,-all)
    set(FORCE_LINK_OFF -Wl,-notall)
  endif("${CMAKE_SYSTEM}" MATCHES IRIX)
  # On Darwin:
  #  -all_load $convenience
  if("${CMAKE_SYSTEM}" MATCHES Darwin)
    set(FORCE_LINK_ON -Wl,-force_load)
    set(FORCE_LINK_OFF "")
  endif("${CMAKE_SYSTEM}" MATCHES Darwin)
  # On Solaris 2: ??

  # TODO: come up with a windows solution
endif(UNIX)

include_directories(${PROTOBUF_INCLUDE_DIRS})
include_directories("." ${BUILD_DIR})

# PROTOCOL BUFFERS -------------------------------------------------------------

function(ROOTREPO_PROTOBUF_GENERATE_CPP RELATIVE_DIR SRCS HDRS)
    if(NOT ARGN)
    message(SEND_ERROR "Error: PROTOBUF_GENERATE_CPP() called without any proto files")
    return()
  endif()

  set(${SRCS})
  set(${HDRS})
  foreach(FIL ${ARGN})
    get_filename_component(ABS_FIL ${FIL} ABSOLUTE)
    get_filename_component(FIL_WE ${FIL} NAME_WE)

    list(APPEND ${SRCS} "${CMAKE_CURRENT_BINARY_DIR}/${FIL_WE}.pb.cc")
    list(APPEND ${HDRS} "${CMAKE_CURRENT_BINARY_DIR}/${FIL_WE}.pb.h")

    add_custom_command(
      OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/${FIL_WE}.pb.cc"
             "${CMAKE_CURRENT_BINARY_DIR}/${FIL_WE}.pb.h"
      COMMAND  ${PROTOBUF_PROTOC_EXECUTABLE}
      ARGS --cpp_out  ${ROOT_SRC_DIR} --python_out ${ROOT_SRC_DIR} -I ${ROOT_SRC_DIR} ${ABS_FIL}
      DEPENDS ${ABS_FIL}
      COMMENT "Running C++ protocol buffer compiler on ${FIL}"
      VERBATIM )
  endforeach()

  set_source_files_properties(${${SRCS}} ${${HDRS}} PROPERTIES GENERATED TRUE)
  set(${SRCS} ${${SRCS}} PARENT_SCOPE)
  set(${HDRS} ${${HDRS}} PARENT_SCOPE)
endfunction()

# Internal function: search for normal library as well as a debug one
#    if the debug one is specified also include debug/optimized keywords
#    in *_LIBRARIES variable
function(_protobuf_find_libraries name filename)
   find_library(${name}_LIBRARY
       NAMES ${filename}
       PATHS ${PROTOBUF_SRC_ROOT_FOLDER}/vsprojects/Release)
   mark_as_advanced(${name}_LIBRARY)

   find_library(${name}_LIBRARY_DEBUG
       NAMES ${filename}
       PATHS ${PROTOBUF_SRC_ROOT_FOLDER}/vsprojects/Debug)
   mark_as_advanced(${name}_LIBRARY_DEBUG)

   if(NOT ${name}_LIBRARY_DEBUG)
      # There is no debug library
      set(${name}_LIBRARY_DEBUG ${${name}_LIBRARY} PARENT_SCOPE)
      set(${name}_LIBRARIES     ${${name}_LIBRARY} PARENT_SCOPE)
   else()
      # There IS a debug library
      set(${name}_LIBRARIES
          optimized ${${name}_LIBRARY}
          debug     ${${name}_LIBRARY_DEBUG}
          PARENT_SCOPE
      )
   endif()
endfunction()

# ADD SUBDIRS LAST -------------------------------------------------------------
add_subdirectory(fab)
add_subdirectory(util)
