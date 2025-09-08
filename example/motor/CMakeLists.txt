cmake_minimum_required(VERSION 3.16)
project(motor_ctl_go2)

list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
find_package(unitree_sdk2 REQUIRED)


add_executable(${PROJECT_NAME} motor_ctl_go2.cpp)
target_link_libraries(${PROJECT_NAME}  unitree_sdk2)

SET(CMAKE_BUILD_TYPE Release)
