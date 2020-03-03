cmake_minimum_required(VERSION 3.1)
project(pymesh)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

file(GLOB srcs "*.h" "*.cpp")
#
#add_library(pymesh_tiny
#		MshLoader.h
#		MshLoader.cpp
#		MshSaver.h
#		MshSaver.cpp
#		Exception.h
#)
#target_include_directories(pymesh PUBLIC ..)
#
## Link against igl_common to draw Eigen and C++11
#target_link_libraries(pymesh_tiny PUBLIC igl_common)

add_library(pymesh ${srcs})
