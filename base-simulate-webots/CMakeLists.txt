cmake_minimum_required(VERSION 3.5)
project(app_base_webots)

link_directories(/usr/local/webots/lib/controller/)
include_directories(/usr/local/webots/include/controller/c/)
include_directories(./src)

add_executable(base_webots
    ./src/main.c
    ./src/motor.c
    ./src/odom.c
    ./src/lidar.c
    ./src/camera.c
    )

target_compile_options(base_webots PUBLIC -O1)
target_link_libraries(base_webots ufr Controller m)
install(TARGETS base_webots DESTINATION bin)