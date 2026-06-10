cmake_minimum_required(VERSION 3.5)
project(cpu1_notebook)


add_compile_definitions(ROBOT_TOPIC_CMD_VEL="@new webots @topic cmd_vel")

add_subdirectory(../packages/app_teleop   app_teleop)

