cmake_minimum_required(VERSION 3.22)
project(robot_arm_host C)
set(CMAKE_C_STANDARD 11)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
enable_testing()

# All host static libs (including axis_core) must be position-independent so
# they can later be linked into the SHARED libsimaxis (Task 9) on Linux too.
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../components/axis_core axis_core)

add_library(simcore STATIC sim/motor_model.c sim/bench.c)
target_include_directories(simcore PUBLIC sim)
target_link_libraries(simcore PRIVATE axis_core m)
target_compile_options(simcore PRIVATE -Wall -Wextra -Werror)

# axis_test(name [extra_libs...]) links axis_core by default; pass extra_libs
# (e.g. simcore, which already links axis_core transitively) to avoid duplicates.
function(axis_test name)
    add_executable(${name} tests/${name}.c)
    if(ARGN)
        target_link_libraries(${name} PRIVATE ${ARGN})
    else()
        target_link_libraries(${name} PRIVATE axis_core)
    endif()
    target_compile_options(${name} PRIVATE -Wall -Wextra -Werror)
    add_test(NAME ${name} COMMAND ${name})
endfunction()

axis_test(test_smoke)
axis_test(test_pid)
axis_test(test_protocol)
axis_test(test_config_table)
axis_test(test_axis_states)
axis_test(test_axis_motion)
axis_test(test_axis_homing)
axis_test(test_motor_model simcore)
axis_test(test_bench simcore)

# Shared library for the Python simulator (ctypes): build/host/libsimaxis.{dylib,so}
add_library(simaxis SHARED sim/simlib_api.c)
target_link_libraries(simaxis PRIVATE simcore axis_core m)
target_compile_options(simaxis PRIVATE -Wall -Wextra -Werror)
set_target_properties(simaxis PROPERTIES
    C_VISIBILITY_PRESET hidden
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
