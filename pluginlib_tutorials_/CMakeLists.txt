cmake_minimum_required(VERSION 3.0.2)
project(pluginlib_tutorials_)
find_package(catkin REQUIRED COMPONENTS
  pluginlib
  roscpp
)

#Here we are declaring 
#this package has a library
# called polygon_plugins
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES polygon_plugins
  CATKIN_DEPENDS pluginlib roscpp
  #DEPENDS system_lib
)
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

#the name of this library
#is important for exporting
#this plugin via package.xml
add_library(
  polygon_plugins src/polygon_plugins.cpp
#   src/${PROJECT_NAME}/pluginlib_tutorials_.cpp
)
target_link_libraries(polygon_plugins ${catkin_LIBRARIES})

add_executable(polygon_loader src/polygon_loader.cpp)
target_link_libraries(polygon_loader ${catkin_LIBRARIES})

#install(TARGETS polygon_plugins polygon_loader
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
#
#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.h"
#  PATTERN ".svn" EXCLUDE
#)
#
#install(FILES polygon_plugins.xml
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
