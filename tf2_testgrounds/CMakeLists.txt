cmake_minimum_required(VERSION 3.0.2)
project(tf2_testgrounds)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf2
  tf2_ros
  message_generation
  std_msgs
  geometry_msgs
  nav_msgs
  tf2_eigen
  tf2_msgs
  tf2_geometry_msgs
  std_srvs
  sensor_msgs
)

add_service_files(
   FILES
   SendTarget.srv
   box_and_target_position.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
   geometry_msgs
   nav_msgs
   sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include/tf2_testgrounds/
  LIBRARIES tf2_testgrounds
  CATKIN_DEPENDS roscpp rospy tf2 tf2_ros message_generation std_msgs geometry_msgs nav_msgs tf2_eigen tf2_msgs tf2_geometry_msgs std_srvs sensor_msgs
)

add_executable(tf2_node src/tf2_node.cpp)
target_link_libraries(tf2_node ${catkin_LIBRARIES})
add_dependencies(tf2_node tf2_testgrounds_gencpp)

add_executable(fake_publisher src/fake_publisher.cpp)
target_link_libraries(fake_publisher ${catkin_LIBRARIES})
add_dependencies(fake_publisher tf2_testgrounds_gencpp)

add_executable(fake_client src/fake_client.cpp)
target_link_libraries(fake_client ${catkin_LIBRARIES})
add_dependencies(fake_client tf2_testgrounds_gencpp)

add_executable(data_associator src/data_associator.cpp)
target_link_libraries(data_associator ${catkin_LIBRARIES})
add_dependencies(data_associator tf2_testgrounds_gencpp)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
