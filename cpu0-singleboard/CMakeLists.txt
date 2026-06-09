cmake_minimum_required(VERSION 3.5)
project(app_base_pioneer)


set(MQTT_HOST 177.153.62.174)
add_compile_definitions(ROBOT_TOPIC_CMD_VEL="@new mqtt @coder msgpack @topic /pioneer/cmd_vel @host ${MQTT_HOST}")
add_compile_definitions(ROBOT_TOPIC_ODOM="@new mqtt @coder msgpack @topic /pioneer/odom @host ${MQTT_HOST}")
add_compile_definitions(ROBOT_TOPIC_CAMERA_RGB="@new mqtt @coder msgpack @topic /pioneer/camera_rgb @host ${MQTT_HOST}")


add_subdirectory(../packages/app_base_pioneer    app_base_pioneer)
add_subdirectory(../packages/app_lidar_rplidar   app_lidar_rplidar)
add_subdirectory(../packages/app_camera_opencv   app_camera_opencv)
