# ==============================================================================
# Project Configuration
# ==============================================================================
cmake_minimum_required(VERSION 3.10)
project(kuavo_common)

# ==============================================================================
# Catkin Configuration
# ==============================================================================
find_package(catkin REQUIRED)

# Find Eigen3
find_package(Eigen3 REQUIRED)
if(NOT EIGEN3_FOUND)
    message(FATAL_ERROR "Eigen3 is required but not found!")
endif()

# Declare catkin package
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    DEPENDS EIGEN3
)

# ==============================================================================
# Build Settings
# ==============================================================================
# Enforce C++17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Generate compile_commands.json for tooling
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ==============================================================================
# Include Directories
# ==============================================================================
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

# ==============================================================================
# Source Files
# ==============================================================================
set(SRC_OBJECTS
    src/kuavo_common.cpp
    src/common/kuavo_settings.cpp
    src/common/utils.cpp
    include/${PROJECT_NAME}/common/json.hpp
    include/${PROJECT_NAME}/common/json_config_reader.hpp
)

# ==============================================================================
# Library Target
# ==============================================================================
# Create shared library
add_library(${PROJECT_NAME} SHARED ${SRC_OBJECTS})

# Link dependencies
target_link_libraries(${PROJECT_NAME} 
    ${catkin_LIBRARIES}
    ${EIGEN3_LIBRARIES}
    stdc++fs
)

# ==============================================================================
# Testing
# ==============================================================================
enable_testing()

# Add test executable
add_executable(${PROJECT_NAME}_test tests/kuavo_common_test.cpp)
target_link_libraries(${PROJECT_NAME}_test
    PRIVATE
        ${PROJECT_NAME}
        gtest
        gtest_main
)

# Register test
add_test(
    NAME KuavoCommonTest
    COMMAND ${PROJECT_NAME}_test
)

# ==============================================================================
# Installation
# ==============================================================================

# Install library
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
