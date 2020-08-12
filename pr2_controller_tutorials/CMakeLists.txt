cmake_minimum_required(VERSION 3.0.2)
project(pr2_controller_tutorials)

find_package(catkin REQUIRED COMPONENTS
  pluginlib
  pr2_controller_interface
  pr2_mechanism_model
  controller_interface
  std_srvs
  message_generation
)

add_service_files(
  FILES
  SetAmplitude.srv
)

add_message_files(
  FILES
  MyStateMessage.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pr2_controller_tutorials
  CATKIN_DEPENDS pluginlib pr2_controller_interface pr2_mechanism_model message_runtime std_srvs
  #  DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_library(pr2_controller_tutorials
   src/my_controller_file.cpp
)
target_link_libraries(pr2_controller_tutorials ${catkin_LIBRARIES})

install(TARGETS pr2_controller_tutorials
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(FILES controller_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
