cmake_minimum_required(VERSION 2.4.6)

project(ras_lab1_open_loop_control)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs ras_arduino_msgs)

catkin_package(CATKIN_DEPENDS roscpp ras_arduino_msgs std_msgs)

add_executable(open_loop_controller src/open_loop_controller.cpp)
add_dependencies(open_loop_controller ${catkin_EXPORTED_TARGETS})
target_link_libraries(open_loop_controller ${catkin_LIBRARIES})
