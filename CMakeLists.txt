cmake_minimum_required(VERSION 3.10)


# Project name
project(IMU-GNSS-EKF-Localization)


# Specify the C++ standard
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "-g -O3")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")

file(GLOB SOURCES "src/*.cpp")
message(STATUS "SOURCE_DIRS: ${SOURCES}")

include_directories(${PROJECT_SOURCE_DIR}/include/)
message(STATUS "INCLUDE_DIRS: ${PROJECT_SOURCE_DIR}/include")


# Manually specify the Eigen3 directory
set(EIGEN3_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/Thirdparty/eigen-3.4.0")
include_directories(${EIGEN3_INCLUDE_DIR})

message(STATUS "The EIGEN header: ${EIGEN3_INCLUDE_DIR}")


# find_package()

# Add the executable
add_executable(EKF_Localization ${SOURCES})


# target_link_libraries()