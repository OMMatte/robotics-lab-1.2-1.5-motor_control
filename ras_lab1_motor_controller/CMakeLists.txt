cmake_minimum_required(VERSION 2.4.6)
project(ras_lab1_motor_controller)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)
catkin_package()

add_executable(motor_controller src/motor_controller.cpp)
target_link_libraries(motor_controller ${catkin_LIBRARIES})
