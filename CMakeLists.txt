cmake_minimum_required(VERSION 2.8.3)
project(my_spot)


#warning: variable CMAKE_CXX_FLAGS is modified
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")


# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    gazebo_ros
  LIBRARIES ${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
 
include_directories(${catkin_INCLUDE_DIRS})

