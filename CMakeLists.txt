cmake_minimum_required(VERSION 2.8.9)
project(navigator)
find_package (Threads)

file(GLOB SOURCES "source/src/*.cpp" "source/include/*.hpp" "../pathfinding/source/src/*.cpp" "../pathfinding/source/include/*.hpp" "../map/source/src/*.cpp" "../map/source/include/*.hpp" "../pilot/source/src/*.cpp" "../pilot/source/include/*.hpp" "../adt/source/src/*.cpp" "../adt/source/include/*.hpp" "../sharedobjects/source/src/*.cpp" "../sharedobjects/source/include/*.hpp")
file(GLOB SOURCES_EXAMPLES "examples/*.cpp" "examples/*.hpp")
file(GLOB SOURCES_GTEST "source/src/*.cpp" "source/include/*.hpp" "test/*.cpp" "test/*.hpp" "../pathfinding/source/src/*.cpp" "../pathfinding/source/include/*.hpp" "../map/source/src/*.cpp" "../map/source/include/*.hpp" "../pilot/source/src/*.cpp" "../pilot/source/include/*.hpp" "../adt/source/src/*.cpp" "../adt/source/include/*.hpp" "../sharedobjects/source/src/*.cpp" "../sharedobjects/source/include/*.hpp")
list(REMOVE_ITEM SOURCES_GTEST ${CMAKE_CURRENT_SOURCE_DIR}/source/src/main.cpp)

SET( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -pthread -std=gnu++11 -Wall" )

include_directories(
  ../adt/source/include
  ../sharedobjects/source/include
  ../pilot/source/include
  ../pathfinding/source/include
  ../map/source/include
  ../deps/gtest-1.7.0/include
  ../deps/gtest-1.7.0)

link_directories(../deps/gtest-1.7.0/src)
ADD_LIBRARY(gtest ../deps/gtest-1.7.0/src/gtest-all.cc)
ADD_LIBRARY(gtest_main ../deps/gtest-1.7.0/src/gtest_main.cc)

add_executable(navigator_example ${SOURCES_EXAMPLES} ${SOURCES})
add_executable(navigator_test ${GTEST} ${SOURCES_GTEST})
target_link_libraries(navigator_test gtest gtest_main ${CMAKE_THREAD_LIBS_INIT})
