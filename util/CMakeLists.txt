cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BUILD_DIR}/rootrepo/util)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${BUILD_DIR}/rootrepo/util)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${BUILD_DIR}/rootrepo/util)

PROJECT(util)

SET(util_SRCS
  color_convert.cc
  simple_uri.cc
  ublas_util.cc)
ADD_LIBRARY(util_lib STATIC ${util_SRCS})
