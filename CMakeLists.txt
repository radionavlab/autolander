cmake_minimum_required(VERSION 2.8.3)
project(autolander)

#NOTE: Eigen includes are different in the snapdragon and odroid.  Snap uses Eigen includes, Odroid uses Eigen3 includes.

#list(APPEND CMAKE_INCLUDE_PATH "/opt/ros/kinetic/share/cmake_modules/cmake/Modules/")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  std_msgs
  cmake_modules
  nav_msgs
  mg_msgs
)

find_package(Eigen REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS mg_msgs
    DEPENDS Eigen
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS std_msgs geometry_msgs mg_msgs nav_msgs)

add_executable(autolander_node src/autolander.cpp)
add_dependencies(autolander_node mg_msgs_generate_messages_cpp)

target_link_libraries(autolander
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  )

