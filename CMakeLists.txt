cmake_minimum_required(VERSION 3.16)
project(fpv-sim VERSION 1.0.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Collect all source files
file(GLOB_RECURSE SOURCES "src/*.cpp")

# Create the executable
add_executable(${PROJECT_NAME} ${SOURCES})

# Add include directory so #include "core/messages/messages.hpp" works
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/include)
