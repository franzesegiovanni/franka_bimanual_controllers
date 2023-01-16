cmake_minimum_required(VERSION 3.4)
project(franka_bimanual_teaching)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/bimanual.cfg
  cfg/dualteaching.cfg
)

catkin_package(
  LIBRARIES franka_bimanual_teaching
  CATKIN_DEPENDS
    dynamic_reconfigure
)
