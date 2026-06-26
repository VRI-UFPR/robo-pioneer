cmake_minimum_required(VERSION 3.5)
project(app_view_lidar)

find_package(OpenCV)

add_executable(view_lidar ./src/main.cpp)
target_link_libraries(view_lidar ufr ${OpenCV_LIBS})
install(TARGETS view_lidar DESTINATION bin)
