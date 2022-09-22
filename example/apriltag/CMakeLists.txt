cmake_minimum_required(VERSION 3.1)
project(maytag-apriltag)

if (NOT CMAKE_BUILD_TYPE)
	message(STATUS "No build type selected, default to Release")
	set(CMAKE_BUILD_TYPE "Release")
endif()

set(CMAKE_CXX_STANDARD 14)

add_executable(${PROJECT_NAME} main.cpp)

include_directories("../../include")

find_package(OpenCV 4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

find_package(apriltag REQUIRED)
target_link_libraries(${PROJECT_NAME} apriltag::apriltag)