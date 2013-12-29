cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake-modules)
project(AlgoritmoSegmentacion)

find_package(PCL 1.2 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS} ${OpenCV_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS} ${OpenCV_DEFINITIONS})

add_executable (AS src/pinhole.cpp src/map.cpp src/conversion.cpp src/save.cpp src/visualize.cpp src/learning.cpp src/roi.cpp src/extract_colour.cpp src/colour_analysis.cpp src/statistics.cpp src/filtering.cpp src/main.cpp src/segmentation.cpp src/algebra.cpp)
target_link_libraries (AS ${PCL_LIBRARIES} ${OpenCV_LIBS} )
add_subdirectory(src)
