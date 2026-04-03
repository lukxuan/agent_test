include(FetchContent)
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        v1.14.0
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# Test helpers
add_library(test_utils INTERFACE)
target_include_directories(test_utils INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/..)

function(add_parking_test name sources)
    add_executable(${name} ${sources})
    target_link_libraries(${name} PRIVATE auto_parking_lib GTest::gtest_main)
    add_test(NAME ${name} COMMAND ${name})
endfunction()

add_parking_test(test_vehicle_model    test_vehicle_model.cpp)
add_parking_test(test_path_planner     test_path_planner.cpp)
add_parking_test(test_motion_controller test_motion_controller.cpp)
add_parking_test(test_parking_scenario test_parking_scenario.cpp)
add_parking_test(test_parking_system   test_parking_system.cpp)
