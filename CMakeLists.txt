cmake_minimum_required(VERSION 3.12)
project(CoM)
find_package(PCL 1.2 REQUIRED)
find_package(OpenCV 3.1 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(CMAKE_CXX_STANDARD 14)

add_executable(CoM main.cpp VisualHull.cpp VisualHull.h Seg.cpp Seg.h cameraLay.cpp cameraLay.h)
target_link_libraries (CoM ${PCL_LIBRARIES})
target_link_libraries (CoM ${OpenCV_LIBS})