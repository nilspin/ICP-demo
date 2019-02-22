#====================
cmake_minimum_required (VERSION 3.9)
PROJECT(ICP_Demo_gold LANGUAGES C CXX)

# Useful CMake options
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_FLAGS "-std=c++11 -fpermissive")
set(OpenGL_GL_PREFERENCE "GLVND")

find_package(Eigen3 REQUIRED)
find_package(glm REQUIRED)
find_package(SDL2 REQUIRED)
message(STATUS "GLM included at ${GLM_INCLUDE_DIR}")

set(INCLUDE_DIRS  ${INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${GLM_INCLUDE_DIR} ${SDL2_INCLUDE_DIRS})

add_library(Solver SHARED Solver.h Solver.cpp)
add_library(Colormap SHARED igl_inline.h colormap.h )
set_target_properties(Colormap PROPERTIES LINKER_LANGUAGE C)

set(LIBS ${LIBS} Colormap ${SDL2_LIBRARIES} Solver )

#specify include directory
include_directories(${INCLUDE_DIRS})

set(SOURCES goldMain.cpp utils.hpp EigenUtil.h common.h termcolor.hpp DebugHelper.hpp)

add_executable(${PROJECT_NAME} ${SOURCES})

#Link all obj files
target_link_libraries(${PROJECT_NAME} ${LIBS} SDL2)

#Now copy shaders to build directory

add_custom_target(copy-runtime-files ALL
    COMMAND cmake -E copy_directory ${CMAKE_SOURCE_DIR}/assets ${CMAKE_BINARY_DIR}/assets
    DEPENDS ${MY_TARGET})
