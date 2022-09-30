cmake_minimum_required(VERSION 3.1)
project(maytag-apriltag)

if (NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE "Release")
endif()

set(CMAKE_CXX_STANDARD 14)

add_executable(${PROJECT_NAME} main.cpp)

#
include_directories("../../include")

# OpenCV.
find_package(OpenCV 4)
if(NOT OpenCV_FOUND)
	find_package(OpenCV 3 REQUIRED)
endif()
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

# AprilTag.
find_package(apriltag REQUIRED)
target_link_libraries(${PROJECT_NAME} apriltag::apriltag)