cmake_minimum_required(VERSION 2.8.3)
project(car_maneuver_recovery)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  costmap_2d
  nav_core
  pluginlib
  base_local_planner
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    car_maneuver_recovery
  CATKIN_DEPENDS
    roscpp
    tf
    costmap_2d
    nav_core
    pluginlib
    base_local_planner
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(car_maneuver_recovery
  src/${PROJECT_NAME}.cpp
)

target_link_libraries(
  car_maneuver_recovery
  ${catkin_LIBRARIES}
)

install(
  FILES car_maneuver_recovery_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
