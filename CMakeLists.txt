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

# Add the executable
add_executable(EKF_Localization ${SOURCES})


# Manually specify the Eigen3-3.4.0 directory
set(EIGEN3_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/Thirdparty/eigen-3.4.0")
include_directories(${EIGEN3_INCLUDE_DIR})

message(STATUS "The EIGEN header: ${EIGEN3_INCLUDE_DIR}")

# Manually specify the yaml-cpp-0.7.0 directory
set(YAML_CPP_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/Thirdparty/yaml-cpp-0.7.0")
add_subdirectory(${YAML_CPP_INCLUDE_DIR})

message(STATUS "The yaml_cpp header: ${YAML_CPP_INCLUDE_DIR}")

target_link_libraries(EKF_Localization yaml-cpp)

# yaml-cpp-0.7.0
# add_subdirectory(ThirdParty/yaml-cpp-0.7.0)
# target_link_libraries(EKF_Localization yaml-cpp)


# find_package()




# target_link_libraries()