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

# Find SLICOT
find_library(SLICOT_LIB NAMES slicot HINTS /opt/homebrew/opt/slicot/lib /usr/local/opt/slicot/lib REQUIRED)
find_program(GFORTRAN_EXECUTABLE NAMES gfortran REQUIRED)
find_library(GFORTRAN_LIB NAMES gfortran HINTS /opt/homebrew/opt/gcc/lib/gcc/current /usr/local/opt/gcc/lib/gcc/current REQUIRED)
find_library(QUADMATH_LIB NAMES quadmath HINTS /opt/homebrew/opt/gcc/lib/gcc/current /usr/local/opt/gcc/lib/gcc/current REQUIRED)

# Collect all source files
file(GLOB_RECURSE SOURCES "src/*.cpp")
list(FILTER SOURCES EXCLUDE REGEX "src/simulation/control/linear_quadratic/linear_quadratic\\.cpp$")

set(SLICOT_CARE_WRAPPER_OBJ ${CMAKE_CURRENT_BINARY_DIR}/slicot_care_wrapper.o)
add_custom_command(
    OUTPUT ${SLICOT_CARE_WRAPPER_OBJ}
    COMMAND ${GFORTRAN_EXECUTABLE} -c ${CMAKE_SOURCE_DIR}/src/simulation/control/linear_quadratic/slicot_care_wrapper.f90 -o ${SLICOT_CARE_WRAPPER_OBJ}
    DEPENDS ${CMAKE_SOURCE_DIR}/src/simulation/control/linear_quadratic/slicot_care_wrapper.f90
    VERBATIM
)
set_source_files_properties(${SLICOT_CARE_WRAPPER_OBJ} PROPERTIES EXTERNAL_OBJECT TRUE GENERATED TRUE)

# Standalone CARE library
add_library(control_care STATIC
    src/simulation/control/linear_quadratic/linear_quadratic.cpp
    ${SLICOT_CARE_WRAPPER_OBJ}
)
target_include_directories(control_care PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(control_care PUBLIC Eigen3::Eigen ${SLICOT_LIB} ${GFORTRAN_LIB} ${QUADMATH_LIB})

# Create the executable
add_executable(${PROJECT_NAME} ${SOURCES})

# Add include directory
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/include)

# Link Eigen, CppAD/Ipopt, and nlohmann_json to the executable
target_link_libraries(${PROJECT_NAME} 
    control_care
    Eigen3::Eigen
    PkgConfig::CPPAD
    nlohmann_json::nlohmann_json
)
