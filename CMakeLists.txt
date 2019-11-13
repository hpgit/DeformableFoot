cmake_minimum_required(VERSION 3.12)
project(DeformableFoot)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
set(CMAKE_CXX_STANDARD 11)

set(OpenGL_GL_PREFERENCE GLVND)

add_subdirectory(pymesh)
add_subdirectory(engine)
add_subdirectory(render)

find_package(Eigen3 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(flann REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OPENGL_INCLUDE_DIR})
include_directories(${FLANN_INCLUDE_DIRS})

add_executable(pd_example projective_dynamics_example.cpp)
add_executable(rigid_example rigid_dynamics_example.cpp)

target_link_libraries(pd_example pymesh engine render ${OPENGL_gl_LIBRARY} ${GLUT_LIBRARY} ${FLANN_LIBRARIES})
target_link_libraries(rigid_example pymesh engine render ${OPENGL_gl_LIBRARY} ${GLUT_LIBRARY} ${FLANN_LIBRARIES})