cmake_minimum_required(VERSION 3.16)
project(flight-sim VERSION 1.0.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Eigen3
find_package(Eigen3 CONFIG REQUIRED)

# Find pkg-config packages
find_package(PkgConfig REQUIRED)
pkg_check_modules(CPPAD REQUIRED IMPORTED_TARGET cppad)

# Find nlohmann_json
find_package(nlohmann_json CONFIG REQUIRED)

# Collect all source files
file(GLOB_RECURSE SOURCES "src/*.cpp")

# Create the executable
add_executable(${PROJECT_NAME} ${SOURCES})

# Add include directory
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/include)

# Link Eigen, CppAD/Ipopt, and nlohmann_json to the executable
target_link_libraries(${PROJECT_NAME} 
    Eigen3::Eigen
    PkgConfig::CPPAD
    nlohmann_json::nlohmann_json
)
