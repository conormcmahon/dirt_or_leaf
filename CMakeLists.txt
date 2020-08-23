cmake_minimum_required(VERSION 2.8.3)

project(pcl_test)

# Point Cloud Library (PCL)
find_package(PCL 1.09 REQUIRED COMPONENTS)
include_directories(include ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Computational Geometry Algorithms Library (CGAL)
set(CMAKE_BUILD_TYPE "Release")
find_package(CGAL QUIET COMPONENTS)
if(NOT CGAL_FOUND)
  message(STATUS "This project requires CGAL and will not be compiled.")
  return()
endif()

# Boost and its components
find_package( Boost REQUIRED )
if ( NOT Boost_FOUND )
  message(STATUS "This project requires the Boost library, and will not be compiled.")
  return()
endif()


add_library(las_classifier src/las_classifier.cpp)
target_link_libraries(las_classifier ${PCL_LIBRARIES} gdal ${CGAL_LIBRARIES})
#add_library(las_filtering src/las_filtering.cpp)
#target_link_libraries(las_filtering ${PCL_LIBRARIES} gdal ${CGAL_LIBRARIES})
add_library(las_surfacing src/las_surfacing.cpp)
target_link_libraries(las_surfacing ${PCL_LIBRARIES} gdal ${CGAL_LIBRARIES})

add_executable(pcl_test src/pcl_test.cpp)
target_link_libraries(pcl_test ${PCL_LIBRARIES} gdal ${CGAL_LIBRARIES})