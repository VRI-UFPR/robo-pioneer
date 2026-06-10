cmake_minimum_required(VERSION 3.5)
project(app_teleop)

# Teleop
add_executable(teleop ./src/main.c)
target_link_libraries(teleop ufr)
install(TARGETS teleop DESTINATION bin)
