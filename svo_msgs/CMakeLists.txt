cmake_minimum_required(VERSION 2.8.3)
project(svo_msgs)

# search for everything we need to build the messages, dont forget the message_generation
find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  sensor_msgs
)

# search for all msg files
FILE(GLOB messages_to_build RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/msg"
    "${CMAKE_CURRENT_SOURCE_DIR}/msg/*.msg")

# notify catkin to look at the previously found msg files
add_message_files(
  FILES
  ${messages_to_build}
)

# build the header files from the msg files, and notify catkin about the dependencies
generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
)

# export the dependencis of this package for who ever depends on us
catkin_package(
  CATKIN_DEPENDS message_runtime geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)