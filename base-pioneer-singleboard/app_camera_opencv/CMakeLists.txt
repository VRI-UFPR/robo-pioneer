cmake_minimum_required(VERSION 3.5)
project(camera_opencv)


find_package(OpenCV)

# Example 1: simple
add_executable(camera_opencv ./src/main.cpp)
target_link_libraries(camera_opencv ufr ${OpenCV_LIBS})
install(TARGETS camera_opencv DESTINATION bin)
