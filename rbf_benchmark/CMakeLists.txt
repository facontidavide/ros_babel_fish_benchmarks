cmake_minimum_required(VERSION 2.8.3)
project(rbf_benchmark)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING
      "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
endif(NOT CMAKE_BUILD_TYPE)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED geometry_msgs ros_babel_fish ros_type_introspection nav_msgs tf)

## System dependencies are found with CMake's conventions
find_package(benchmark REQUIRED)

set(LIBRARIES
  ${catkin_LIBRARIES}
  benchmark
)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rbf_benchmark
  CATKIN_DEPENDS geometry_msgs ros_babel_fish ros_type_introspection nav_msgs tf
  DEPENDS benchmark
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${benchmark_INCLUDE_DIRS}
)

add_executable(message_lookup src/message_lookup.cpp)
target_link_libraries(message_lookup ${LIBRARIES} pthread)

add_executable(message_translation src/message_translation.cpp)
target_link_libraries(message_translation ${LIBRARIES} pthread)

#############
## Install ##
#############

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
install(TARGETS message_lookup message_translation
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


