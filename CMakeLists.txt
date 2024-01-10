cmake_minimum_required(VERSION 3.0.2)
project(img_compressor_v2)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

add_message_files(DIRECTORY msg FILES BinarySplit.msg)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime rospy std_msgs)

include_directories(${catkin_INCLUDE_DIRS})

catkin_install_python(PROGRAMS
  scripts/img_compressor_node_v2.py
  scripts/img_joiner_node_v2.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)