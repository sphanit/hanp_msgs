cmake_minimum_required(VERSION 2.8.3)
project(hanp_layer_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
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
    message_runtime
    geometry_msgs
)
