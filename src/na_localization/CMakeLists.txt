cmake_minimum_required(VERSION 2.8.3)
#project(sfast_lio)
project(na_localization)

SET(CMAKE_BUILD_TYPE "Release")

#add_definitions(-DENABLE_SENSOR_FAULT)

ADD_COMPILE_OPTIONS(-std=c++14 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set( CMAKE_CXX_FLAGS "-std=c++14 -O3" )

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")


set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" )
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread -std=c++0x -std=c++14 -fexceptions")

message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)" )
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processer number:  ${N}")
  if(N GREATER 4)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=3)
    message("core for MP: 3")
  elseif(N GREATER 3)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=2)
    message("core for MP: 2")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

list(APPEND CMAKE_MODULE_PATH "usr/share/cmake/geographiclib")

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  pcl_ros
  tf
  livox_ros_driver
  message_generation
  eigen_conversions
)

find_package(Eigen3 REQUIRED)
#find_package(PCL 1.8 REQUIRED)
find_package(PCL REQUIRED)
#find_package( Sophus REQUIRED )

find_package( GTSAMCMakeTools )
find_package(GTSAM REQUIRED QUIET)
find_package (GeographicLib REQUIRED)

message(Eigen: ${EIGEN3_INCLUDE_DIR})

include_directories(
	${catkin_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  #${Sophus_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  ${GeographicLib_INCLUDE_DIRS}
  include)
  include_directories(${PROJECT_SOURCE_DIR}/thirdparty/sophus)

include_directories(${PROJECT_SOURCE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/alg)
include_directories(${PROJECT_SOURCE_DIR}/alg/common)
include_directories(${PROJECT_SOURCE_DIR}/alg/common/spdlog)
include_directories(${PROJECT_BINARY_DIR})

INCLUDE_DIRECTORIES(/usr/include/opencv4/)

add_message_files(
  FILES
  Pose6D.msg
  rtk_pos_raw.msg
  rtk_heading_raw.msg
  sensor_vaild.msg

)

add_service_files(
  DIRECTORY srv
  FILES
  save_map.srv
)

generate_messages(
 DEPENDENCIES
 geometry_msgs
 std_msgs
 sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs message_runtime
  DEPENDS EIGEN3 PCL GTSAM
  INCLUDE_DIRS
)

set(COMMON common)
set(RELOC_SERVICE reloc_service)
set(MATCH_RATE_CAL match_rate_cal)
set(LOCAL_MAP_EXTRACT local_map_extract)
set(PLUGINS_RELOC reloc_plugin)

add_subdirectory(alg)

add_subdirectory(plugins)

add_executable(fastlio_mapping src/laserMapping.cpp include/ikd-Tree/ikd_Tree.cpp src/preprocess.cpp include/map_management.cpp)
target_link_libraries(fastlio_mapping ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PYTHON_LIBRARIES}   gtsam ${GeographicLib_LIBRARIES}
${RELOC_SERVICE} ${PLUGINS_RELOC}) 
target_include_directories(fastlio_mapping PRIVATE ${PYTHON_INCLUDE_DIRS})
