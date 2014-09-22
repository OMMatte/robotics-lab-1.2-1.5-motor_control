cmake_minimum_required(VERSION 2.4.6)
project(ras_lab1_open_loop_control)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)
catkin_package()

add_executable(open_loop_controller src/open_loop_controller.cpp)
target_link_libraries(open_loop_controller ${catkin_LIBRARIES})
