cmake_minimum_required(VERSION 2.8.3)
project(rbf_benchmark)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED geometry_msgs ros_babel_fish)

## System dependencies are found with CMake's conventions
find_package(benchmark REQUIRED)

set(LIBRARIES
  ${catkin_LIBRARIES}
  benchmark
)

# Optionally benchmark against ros_type_introspeciton
find_package(ros_type_introspection)

if (ros_type_introspection_FOUND)
  find_package(topic_tools REQUIRED)
  set(LIBRARIES
    ${LIBRARIES}
    ${ros_type_introspection_LIBRARIES}
    ${topic_tools_LIBRARIES}
  )
  add_definitions(-DENABLE_RTI)
endif()

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rbf_benchmark
  CATKIN_DEPENDS geometry_msgs ros_babel_fish
#  DEPENDS system_lib
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
target_link_libraries(message_lookup ${LIBRARIES})

add_executable(message_translation src/message_translation.cpp)
target_link_libraries(message_translation ${LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
install(TARGETS message_lookup message_translation
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
