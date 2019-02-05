cmake_minimum_required(VERSION 2.8.3)
project(pid_tuner)

find_package(catkin REQUIRED
  roscpp
  std_msgs
  rospy
  geometry_msgs
  nav_msgs
  tf
  prius_msgs
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(CATKIN_DEPENDS 
)

add_executable(pid_tuner src/pid_tuner.cpp)
add_executable(pid_test src/pid_test.cpp)

target_link_libraries(pid_tuner
  ${catkin_LIBRARIES}
)
target_link_libraries(pid_test
  ${catkin_LIBRARIES}
)



