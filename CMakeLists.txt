cmake_minimum_required(VERSION 2.8.3)
project(asr_cyberglove_visualization)

find_package(catkin REQUIRED COMPONENTS
  asr_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

catkin_package(
  CATKIN_DEPENDS asr_msgs roscpp rospy sensor_msgs std_msgs tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(test_glove_fob src/test_glove_fob.cpp)
add_dependencies(test_glove_fob
	asr_msgs_generate_messages_cpp
)
target_link_libraries(test_glove_fob ${catkin_LIBRARIES})

add_executable(test_glove src/test_glove.cpp)
add_dependencies(test_glove
	asr_msgs_generate_messages_cpp
)
target_link_libraries(test_glove ${catkin_LIBRARIES})

add_executable(test_fob src/test_fob.cpp)
add_dependencies(test_fob
	asr_msgs_generate_messages_cpp
)
target_link_libraries(test_fob ${catkin_LIBRARIES})
