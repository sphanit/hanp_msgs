cmake_minimum_required(VERSION 2.8.3)
project(hanp_msgs)

set(CMAKE_CXX_COMPILER_ARG1 -std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  geometry_msgs
  tf
  visualization_msgs
)

## add message, serviece and action files

add_message_files( DIRECTORY msg
  FILES
    TrackedSegment.msg
    TrackedHuman.msg
    TrackedHumans.msg
    TrackedSegmentType.msg
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
    tf
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
add_dependencies(fake_humans_publisher
  ${PROJECT_NAME}_generate_messages_cpp
)
target_link_libraries(fake_humans_publisher
  ${catkin_LIBRARIES}
)
