#====================
cmake_minimum_required (VERSION 3.9)
PROJECT(ICP_Demo_gold LANGUAGES C CXX)

#static analysis opens big can of worms. keep it closed for now.
#set(CMAKE_CXX_CLANG_TIDY "clang-tidy;-checks=*;-header-filter=.;")

# Useful CMake options
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_FLAGS "-std=c++11 -fpermissive")
set(OpenGL_GL_PREFERENCE "GLVND")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) #ensures CMake creates compile_commands.json
IF( EXISTS "${CMAKE_CURRENT_BINARY_DIR}/compile_commands.json" )
  EXECUTE_PROCESS( COMMAND ${CMAKE_COMMAND} -E copy_if_different
    ${CMAKE_CURRENT_BINARY_DIR}/compile_commands.json
    ${CMAKE_CURRENT_SOURCE_DIR}/compile_commands.json
  )
ENDIF()

find_package(Threads)
find_package(Eigen3 REQUIRED)
find_package(glm REQUIRED)
find_package(SDL2 REQUIRED)
message(STATUS "GLM included at ${GLM_INCLUDE_DIR}")

set(INCLUDE_DIRS  ${INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${GLM_INCLUDE_DIR} ${SDL2_INCLUDE_DIRS})

add_library(Solver SHARED Solver.h Solver.cpp)
#add_library(Colormap SHARED igl_inline.h colormap.h)
add_library(Colormap SHARED colormap.h colormap.cpp) #also check flag
# in colormap.h
set_target_properties(Colormap PROPERTIES LINKER_LANGUAGE C)

set(LIBS ${LIBS} Colormap ${SDL2_LIBRARIES} Solver )

#specify include directory
include_directories(${INCLUDE_DIRS})

set(SOURCES goldMain.cpp utils.hpp EigenUtil.h common.h termcolor.hpp DebugHelper.hpp)

add_executable(${PROJECT_NAME} ${SOURCES})

#Link all obj files
target_link_libraries(${PROJECT_NAME} ${LIBS} SDL2 pthread)

#Now copy shaders to build directory

add_custom_target(copy-runtime-files ALL
    COMMAND cmake -E copy_directory ${CMAKE_SOURCE_DIR}/assets ${CMAKE_BINARY_DIR}/assets
    DEPENDS ${MY_TARGET})
