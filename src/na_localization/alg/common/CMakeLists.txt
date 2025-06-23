cmake_minimum_required(VERSION 2.8.0)

set(CMAKE_BUILD_TYPE "Release")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3 -fPIC")

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -std=c++14 -g2 -O0 -ggdb -fprofile-arcs -ftest-coverage -fPIC")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -std=c++14 -O2 -fPIC")

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

message("EIGEN3_INCLUDE_DIRS = {EIGEN3_INCLUDE_DIRS}")

find_package(catkin REQUIRED COMPONENTS
    roscpp
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

include_directories(include)
include_directories(${PROJECT_SOURCE_DIR})
include_directories(${PROJECT_BINARY_DIR})

file(GLOB_RECURSE ALL_SRCS
    ./log/log.cc
)

if(GSTATIC)
add_library(${COMMON} GSTATIC ${ALL_SRCS})
else()
add_library(${COMMON} SHARED ${ALL_SRCS})
endif()

target_link_libraries(${COMMON}
    ${EIGEN3_LIBRARIES}
    ${PCL_LIBRARIES}
)


