cmake_minimum_required(VERSION 3.9.5)
project(rplidar_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -pedantic)
endif()

set(RPLIDAR_SDK_PATH "./src/sdk/")

FILE(GLOB RPLIDAR_SDK_SRC
  "${RPLIDAR_SDK_PATH}/src/arch/linux/*.cpp"
  "${RPLIDAR_SDK_PATH}/src/hal/*.cpp"
  "${RPLIDAR_SDK_PATH}/src/*.cpp"
)

include_directories(
  ${RPLIDAR_SDK_PATH}/include
  ${RPLIDAR_SDK_PATH}/src
  include
)

# build composition node
add_executable(rplidar 
    src/main.cpp 
    ${RPLIDAR_SDK_SRC})

target_link_libraries(rplidar ufr)
install(TARGETS rplidar DESTINATION bin)