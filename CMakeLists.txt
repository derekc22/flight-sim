cmake_minimum_required(VERSION 3.16)
project(flight-sim VERSION 1.0.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Eigen3
find_package(Eigen3 CONFIG REQUIRED)
find_package(proxsuite CONFIG REQUIRED)
find_package(Threads REQUIRED)

# Find pkg-config packages
find_package(PkgConfig REQUIRED)
pkg_check_modules(CPPAD REQUIRED IMPORTED_TARGET cppad)

# Find nlohmann_json
find_package(nlohmann_json CONFIG REQUIRED)

# Find SDL3
find_package(SDL3 CONFIG REQUIRED)

# Fetch Rerun SDK
include(FetchContent)
FetchContent_Declare(rerun_sdk
    URL https://github.com/rerun-io/rerun/releases/latest/download/rerun_cpp_sdk.zip
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
FetchContent_MakeAvailable(rerun_sdk)

# Fetch spdlog
FetchContent_Declare(spdlog
    GIT_REPOSITORY https://github.com/gabime/spdlog.git
    GIT_TAG v1.17.0
)
FetchContent_MakeAvailable(spdlog)

# Find GoogleTest
find_package(GTest CONFIG REQUIRED)
include(GoogleTest)
enable_testing()
get_target_property(GTEST_INCLUDE_DIRS GTest::gtest INTERFACE_INCLUDE_DIRECTORIES)

# Find SLICOT
find_library(SLICOT_LIB NAMES slicot HINTS /opt/homebrew/opt/slicot/lib /usr/local/opt/slicot/lib REQUIRED)
find_program(GFORTRAN_EXECUTABLE NAMES gfortran REQUIRED)
find_library(GFORTRAN_LIB NAMES gfortran HINTS /opt/homebrew/opt/gcc/lib/gcc/current /usr/local/opt/gcc/lib/gcc/current REQUIRED)
find_library(QUADMATH_LIB NAMES quadmath HINTS /opt/homebrew/opt/gcc/lib/gcc/current /usr/local/opt/gcc/lib/gcc/current REQUIRED)

# Find OpenCV
if(NOT OpenCV_DIR)
    foreach(OPENCV_PREFIX /opt/homebrew/opt/opencv /usr/local/opt/opencv)
        if(EXISTS "${OPENCV_PREFIX}/lib/cmake/opencv4/OpenCVConfig.cmake")
            set(OpenCV_DIR "${OPENCV_PREFIX}/lib/cmake/opencv4")
            break()
        endif()
    endforeach()
endif()
find_package(OpenCV REQUIRED)

# Collect all source files
file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS "src/*.cpp")
file(GLOB_RECURSE IO_CV2_SOURCES CONFIGURE_DEPENDS "src/core/io/camera/*.cpp")
list(FILTER SOURCES EXCLUDE REGEX "src/simulation/control/linear_quadratic/slicot_care\\.cpp$")
list(REMOVE_ITEM SOURCES ${IO_CV2_SOURCES})
set(TESTED_SOURCES ${SOURCES})
list(FILTER TESTED_SOURCES EXCLUDE REGEX "src/main\\.cpp$")
file(GLOB_RECURSE TEST_SOURCES CONFIGURE_DEPENDS "tests/*.cpp")

set(SLICOT_CARE_WRAPPER_OBJ ${CMAKE_CURRENT_BINARY_DIR}/slicot_care_wrapper.o)
add_custom_command(
    OUTPUT ${SLICOT_CARE_WRAPPER_OBJ}
    COMMAND ${GFORTRAN_EXECUTABLE} -ffree-line-length-none -c ${CMAKE_SOURCE_DIR}/src/simulation/control/linear_quadratic/slicot_care_wrapper.f90 -o ${SLICOT_CARE_WRAPPER_OBJ}
    DEPENDS ${CMAKE_SOURCE_DIR}/src/simulation/control/linear_quadratic/slicot_care_wrapper.f90
    VERBATIM
)
set_source_files_properties(${SLICOT_CARE_WRAPPER_OBJ} PROPERTIES EXTERNAL_OBJECT TRUE GENERATED TRUE)

# Standalone CARE library
add_library(control_care STATIC
    src/simulation/control/linear_quadratic/slicot_care.cpp
    ${SLICOT_CARE_WRAPPER_OBJ}
)
target_include_directories(control_care PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(control_care PUBLIC Eigen3::Eigen ${SLICOT_LIB} ${GFORTRAN_LIB} ${QUADMATH_LIB})

# Create the executable
add_library(flight_sim_core STATIC ${TESTED_SOURCES})
add_library(flight_sim_cv2 STATIC ${IO_CV2_SOURCES})
add_executable(${PROJECT_NAME} src/main.cpp)

# Add include directory
target_include_directories(flight_sim_core PRIVATE ${CMAKE_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS})
target_include_directories(flight_sim_cv2 PRIVATE ${CMAKE_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS})
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(flight_sim_core PRIVATE control_care Eigen3::Eigen PkgConfig::CPPAD nlohmann_json::nlohmann_json SDL3::SDL3 rerun_sdk spdlog::spdlog Threads::Threads ${OpenCV_LIBS})
target_link_libraries(flight_sim_core PUBLIC proxsuite::proxsuite)
target_link_libraries(flight_sim_cv2 PRIVATE ${OpenCV_LIBS})

# Link Eigen, CppAD/Ipopt, and nlohmann_json to the executable
target_link_libraries(${PROJECT_NAME} flight_sim_core flight_sim_cv2 control_care Eigen3::Eigen PkgConfig::CPPAD nlohmann_json::nlohmann_json rerun_sdk spdlog::spdlog)

# Test executable
add_executable(flight_sim_tests ${TEST_SOURCES})
target_include_directories(flight_sim_tests SYSTEM BEFORE PRIVATE ${GTEST_INCLUDE_DIRS})
target_include_directories(flight_sim_tests PRIVATE ${CMAKE_SOURCE_DIR}/include ${CMAKE_SOURCE_DIR}/tests ${OpenCV_INCLUDE_DIRS})
target_link_libraries(flight_sim_tests PRIVATE flight_sim_core control_care Eigen3::Eigen PkgConfig::CPPAD nlohmann_json::nlohmann_json rerun_sdk spdlog::spdlog GTest::gtest GTest::gtest_main)
set_target_properties(flight_sim_tests PROPERTIES BUILD_RPATH "$<TARGET_FILE_DIR:GTest::gtest>;$<TARGET_FILE_DIR:GTest::gtest_main>")
gtest_discover_tests(flight_sim_tests)
