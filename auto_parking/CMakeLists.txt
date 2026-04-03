cmake_minimum_required(VERSION 3.16)
project(auto_parking LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Core library
add_library(auto_parking_lib
    src/vehicle_model.cpp
    src/perception.cpp
    src/path_planner.cpp
    src/path_planner_parallel.cpp
    src/path_planner_perpendicular.cpp
    src/path_planner_diagonal.cpp
    src/motion_controller.cpp
    src/parking_scenario.cpp
    src/parking_system.cpp
    src/utils.cpp
)

target_include_directories(auto_parking_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Demo executable
add_executable(auto_parking_demo main.cpp)
target_link_libraries(auto_parking_demo PRIVATE auto_parking_lib)

# Testing
option(BUILD_TESTS "Build tests" ON)
if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
