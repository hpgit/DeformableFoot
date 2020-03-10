cmake_minimum_required(VERSION 3.1)
project(engine)

set(CMAKE_CXX_STANDARD 11)

find_package(Eigen3 REQUIRED)
find_package(OpenGL REQUIRED)

file(GLOB engine_srcs "util/*.h" "util/*.cpp" "rigid/*.cpp" "rigid/*.h" "soft/*.cpp" "soft/*.h" "mixed/*.cpp" "mixed/*.h")

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OPENGL_INCLUDE_DIR})

add_library(engine ${engine_srcs})