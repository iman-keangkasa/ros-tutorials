cmake_minimum_required(VERSION 3.0.2)
project(actionlib_tutorials)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
  roscpp
  rospy
  std_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

add_action_files(
  DIRECTORY action
  FILES Fibonacci.action Averaging.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs std_msgs #and other package used in the action msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES actionlib_tutorials
  CATKIN_DEPENDS actionlib actionlib_msgs message_generation roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable( fibonacci_server src/fibonacci_server.cpp )
target_link_libraries( fibonacci_server ${catkin_LIBRARIES} )
add_dependencies( fibonacci_server ${actionlib_tutorials_EXPORTED_TARGETS} )

add_executable( fibonacci_client src/fibonacci_client.cpp )
target_link_libraries( fibonacci_client ${catkin_LIBRARIES} )
add_dependencies( fibonacci_client ${actionlib_tutorials_EXPORTED_TARGETS} )

add_executable( fibonacci_client2 src/fibonacci_client2.cpp )
target_link_libraries( fibonacci_client2 ${catkin_LIBRARIES} )
add_dependencies( fibonacci_client2 ${actionlib_tutorials_EXPORTED_TARGETS} )

add_executable( averaging_server src/averaging_server.cpp )
target_link_libraries( averaging_server ${catkin_LIBRARIES} )
add_dependencies( averaging_server ${actionlib_tutorials_EXPORTED_TARGETS} )

add_executable( averaging_client src/averaging_client.cpp )
target_link_libraries( averaging_client ${catkin_LIBRARIES} )
add_dependencies( averaging_client ${actionlib_tutorials_EXPORTED_TARGETS} )

add_executable( fibonacci_non_blocking_client src/fibonacci_non_blocking_client.cpp )
target_link_libraries( fibonacci_non_blocking_client ${catkin_LIBRARIES} )
add_dependencies( fibonacci_non_blocking_client ${actionlib_tutorials_EXPORTED_TARGETS} )

add_executable( fibonacci_client_class src/fibonacci_client_class.cpp )
target_link_libraries( fibonacci_client_class ${catkin_LIBRARIES} )
add_dependencies( fibonacci_client_class ${actionlib_tutorials_EXPORTED_TARGETS} )

