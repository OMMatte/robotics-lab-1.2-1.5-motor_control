cmake_minimum_required(VERSION 2.4.6)

project(ras_lab1_motor_controller)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs ras_arduino_msgs geometry_msgs)

catkin_package(CATKIN_DEPENDS roscpp std_msgs ras_arduino_msgs geometry_msgs)

add_executable(motor_controller src/motor_controller.cpp)
add_dependencies(motor_controller ${catkin_EXPORTED_TARGETS})
target_link_libraries(motor_controller ${catkin_LIBRARIES})
