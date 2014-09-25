cmake_minimum_required(VERSION 2.4.6)
project(ras_lab1_cartesian_controllers)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)
catkin_package()

add_executable(line_cartesian_controller src/line_cartesian_controller.cpp)
add_dependencies(line_cartesian_controller ${catkin_EXPORTED_TARGETS})
target_link_libraries(line_cartesian_controller ${catkin_LIBRARIES})

add_executable(circle_cartesian_controller src/circle_cartesian_controller.cpp)
add_dependencies(circle_cartesian_controller ${catkin_EXPORTED_TARGETS})
target_link_libraries(circle_cartesian_controller ${catkin_LIBRARIES})

add_executable(wall_following_controller src/wall_following_controller.cpp)
add_dependencies(wall_following_controller ${catkin_EXPORTED_TARGETS})
target_link_libraries(wall_following_controller ${catkin_LIBRARIES})
