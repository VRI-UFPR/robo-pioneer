cmake_minimum_required(VERSION 3.5)
project(app_pub_camera)

find_package(OpenCV)

# Example 1: simple
add_executable(app_pub_camera ./src/main.cpp)
target_link_libraries(app_pub_camera ufr ${OpenCV_LIBS})
# install(TARGETS app_pub_camera DESTINATION bin)
