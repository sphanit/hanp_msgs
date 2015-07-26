cmake_minimum_required(VERSION 2.8.3)
project(hanp_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  geometry_msgs
  visualization_msgs
)

## add message, serviece and action files

add_message_files( DIRECTORY msg
  FILES
    TrackedHuman.msg
    TrackedHumans.msg
)

## generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    geometry_msgs
)

## declare catkin package
catkin_package(
  CATKIN_DEPENDS
    roscpp
    message_runtime
    geometry_msgs
    visualization_msgs
)



## testing ##

## add helpers for testing
include_directories(
  ${catkin_INCLUDE_DIRS}
)
add_executable(fake_humans_publisher
  test/fake_humans_publisher.cpp
)
target_link_libraries(fake_humans_publisher
  ${catkin_LIBRARIES}
)
