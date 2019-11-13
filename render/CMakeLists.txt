cmake_minimum_required(VERSION 3.1)
project(render)
set(CMAKE_CXX_STANDARD 11)

file(GLOB srcs "*.cpp" "*.h")

set(OpenGL_GL_PREFERENCE GLVND)

find_package(Eigen3 REQUIRED)
find_package(OpenGL REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OPENGL_INCLUDE_DIR})

add_library(render ${srcs})

target_link_libraries(render ${OPENGL_gl_LIBRARY})