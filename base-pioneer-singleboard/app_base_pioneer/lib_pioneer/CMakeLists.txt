cmake_minimum_required(VERSION 3.5)
project(lib_pioneer)

# Public Includes
include_directories(./include)

# Library
add_library(pioneer STATIC
    ./src/pack.c
    ./src/tty.c
    ./src/pioneer_p3dx.c
    )
# install(TARGETS pioneer DESTINATION lib)
# install(FILES include/pioneer_p3dx.h DESTINATION include)

# Example 1: simple
# add_executable(pioneer_simple ./examples/simple.c)
# target_link_libraries(pioneer_simple pioneer)
# install(TARGETS pioneer_simple DESTINATION bin)

# Example 2: teleoperation
# add_executable(pioneer_teleop ./examples/teleop.c)
# target_link_libraries(pioneer_teleop pioneer)
# install(TARGETS pioneer_teleop DESTINATION bin)

# add_executable(controlador ./examples/controlador.c)
# target_link_libraries(controlador ufr)
# install(TARGETS controlador DESTINATION bin)


# Tests
enable_testing()
# set(CMAKE_C_FLAGS "-g -fprofile-arcs -ftest-coverage -O0")
add_executable(test_pack ./tests/ut_pack.c)
target_link_libraries(test_pack pthread gcov)
add_test(NAME test_pack COMMAND ./test_pack)

