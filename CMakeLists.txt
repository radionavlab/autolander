cmake_minimum_required(VERSION 3.1)
project(autolander)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  std_msgs
  cmake_modules
  mg_msgs
  nav_msgs
)

find_package(Eigen REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS geometry_msgs std_msgs mg_msgs nav_msgs
    DEPENDS Eigen
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_executable(autolander_node src/autolander.cpp)
add_dependencies(autolander_node mg_msgs_gencpp)
target_link_libraries(autolander_node ${catkin_LIBRARIES} ${Eigen_LIBRARIES})

