cmake_minimum_required(VERSION 3.0.2)
project(tf2_tutorials)

add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)
if (NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(Boost REQUIRED system filesystem date_time thread)
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  tf
  tf2
  tf2_ros
  tf2_eigen
  tf2_sensor_msgs
  pluginlib
  pluginlib_tutorials_
  #geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES tf2_tutorials
  CATKIN_DEPENDS rospy roscpp tf tf2_ros tf2 tf2_eigen tf2_sensor_msgs pluginlib pluginlib_tutorials_ #geometry_msgs
  DEPENDS EIGEN3
)

include_directories( SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#[IMAN] This is rarely used find_package() is enough
#link_directories(${catkin_LIBRARIES_DIRS})

add_executable(moving_frame src/moving_frame.cpp)
target_link_libraries( moving_frame ${catkin_LIBRARIES})

add_executable(transform_lookup src/transform_lookup.cpp)
target_link_libraries( transform_lookup ${catkin_LIBRARIES})

add_executable( adding_frames src/adding_frames.cpp )
target_link_libraries( adding_frames ${catkin_LIBRARIES} )

add_executable( tf_filters src/tf_filters.cpp )
target_link_libraries( tf_filters ${catkin_LIBRARIES} )

add_executable(polygon_loader src/polygon_loader.cpp)
target_link_libraries( polygon_loader ${catkin_LIBRARIES} )
