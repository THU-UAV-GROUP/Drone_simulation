cmake_minimum_required(VERSION 3.8.2)
project(bprfd)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  sensor_msgs
)

set(PCL_DIR usr/qtcreater/5.14.2/gcc_64/lib/cmake/Qt5)
find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs pcl_ros
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})

add_executable(bprfd src/bprfd_main.cpp 
  src/bprfd.cpp
  src/utils.cpp)

target_link_libraries(bprfd
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

install(DIRECTORY config/
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
