cmake_minimum_required(VERSION 2.4.6)
project(ras_lab1_cartesian_controllers)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)
catkin_package()

add_executable(line_cartesian_controller src/line_cartesian_controller.cpp)
target_link_libraries(line_cartesian_controller ${catkin_LIBRARIES})
