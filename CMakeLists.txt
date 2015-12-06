cmake_minimum_required(VERSION 2.8.3)
project(komodo_coffee_taker)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  gazebo_ros
  moveit_ros_planning_interface
)


## Generate services in the 'srv' folder
add_service_files(
   FILES
   CoffeeTaker.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
  	DEPENDS 
    CATKIN_DEPENDS roscpp rospy std_msgs message_runtime 
    INCLUDE_DIRS 
    LIBRARIES 
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(komodo_coffee_taker src/komodo_coffee_taker.cpp)
target_link_libraries(komodo_coffee_taker ${catkin_LIBRARIES})
add_dependencies(komodo_coffee_taker beginner_tutorials_gencpp)