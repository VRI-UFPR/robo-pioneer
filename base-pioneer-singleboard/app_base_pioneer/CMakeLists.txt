cmake_minimum_required(VERSION 3.5)
project(app_base_pioneer)

include_directories(./lib_pioneer/include)
add_subdirectory(./lib_pioneer)

# Example 1: simple
add_executable(base_pioneer ./src/main.c)
target_link_libraries(base_pioneer pioneer ufr)
install(TARGETS base_pioneer DESTINATION bin)

# Service configuration
# install(FILES ./etc/vri-pioneer.service DESTINATION boot)