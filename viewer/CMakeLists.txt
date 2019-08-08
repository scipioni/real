cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(viewer)
SET(EXECUTABLE_OUTPUT_PATH /lab/real+/bin)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# brain
add_library(brain src/brain.cpp src/brain.hpp)

# PCL
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# OpenCV
FIND_PACKAGE(OpenCV REQUIRED)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIR})

# YARP 
find_package(YARP REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})

add_executable (viewer viewer.cpp)
target_link_libraries (viewer brain ${YARP_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} pcl_filters pcl_visualization ${OpenCV_LIBS})

