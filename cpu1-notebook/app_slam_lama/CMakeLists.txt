cmake_minimum_required(VERSION 3.5)
project(iris_lama_ros)


# Requisitos
# sudo apt install libeigen3-dev libyaml-cpp-dev



# Default to release mode.
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# find_package(iris_lama REQUIRED)
add_subdirectory(./lib_slam_lama)

find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Specify additional locations of header files
include_directories(include ${YAML_CPP_INCLUDE_DIR})
# add_subdirectory(src)

add_executable(graph_slam2d src/graph_slam2d.cpp)
target_link_libraries(graph_slam2d iris_lama ufr ${OpenCV_LIBS})


# Install our lannch files
# install(DIRECTORY launch/
#     DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
#    PATTERN ".svn" EXCLUDE
# )


