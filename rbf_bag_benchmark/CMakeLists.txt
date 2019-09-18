cmake_minimum_required(VERSION 2.8.3)
project(rbf_bag_benchmark)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED ros_babel_fish)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


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
#  LIBRARIES bag_benchmark
  CATKIN_DEPENDS ros_babel_fish
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
)

find_package(ros_type_introspection)
add_executable(benchmark_image_simple_base src/benchmark_image_simple_base.cpp)
target_link_libraries(benchmark_image_simple_base ${catkin_LIBRARIES})

add_executable(benchmark_image_simple_rbf src/benchmark_image_simple_rbf.cpp)
target_link_libraries(benchmark_image_simple_rbf ${catkin_LIBRARIES})

add_executable(benchmark_image_simple_rbf_extractor src/benchmark_image_simple_rbf_extractor.cpp)
target_link_libraries(benchmark_image_simple_rbf_extractor ${catkin_LIBRARIES})

add_executable(benchmark_float_msg_base src/benchmark_float_msg_base.cpp)
target_link_libraries(benchmark_float_msg_base ${catkin_LIBRARIES})

add_executable(benchmark_float_msg_rbf src/benchmark_float_msg_rbf.cpp)
target_link_libraries(benchmark_float_msg_rbf ${catkin_LIBRARIES})

add_executable(benchmark_float_msg_rbf_extractor src/benchmark_float_msg_rbf_extractor.cpp)
target_link_libraries(benchmark_float_msg_rbf_extractor ${catkin_LIBRARIES})

add_executable(benchmark_joint_state_base src/benchmark_joint_state_base.cpp)
target_link_libraries(benchmark_joint_state_base ${catkin_LIBRARIES})

add_executable(benchmark_joint_state_rbf src/benchmark_joint_state_rbf.cpp)
target_link_libraries(benchmark_joint_state_rbf ${catkin_LIBRARIES})

if (ros_type_introspection_FOUND)
  find_package(topic_tools REQUIRED)
  set(RTI_LIBRARIES ${ros_type_introspection_LIBRARIES} ${topic_tools_LIBRARIES})

  add_executable(benchmark_image_simple_rti src/benchmark_image_simple_rti.cpp)
  target_link_libraries(benchmark_image_simple_rti ${catkin_LIBRARIES} ${RTI_LIBRARIES})

  add_executable(benchmark_float_msg_rti src/benchmark_float_msg_rti.cpp)
  target_link_libraries(benchmark_float_msg_rti ${catkin_LIBRARIES} ${RTI_LIBRARIES})

  add_executable(benchmark_joint_state_rti src/benchmark_joint_state_rti.cpp)
  target_link_libraries(benchmark_joint_state_rti ${catkin_LIBRARIES} ${RTI_LIBRARIES})
endif()

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
install(TARGETS
  benchmark_image_simple_base
  benchmark_image_simple_rbf
  benchmark_image_simple_rbf_extractor
  benchmark_float_msg_base
  benchmark_float_msg_rbf
  benchmark_float_msg_rbf_extractor
  benchmark_joint_state_base
  benchmark_joint_state_rbf
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if (ros_type_introspection_FOUND)
  install(TARGETS
    benchmark_image_simple_rti
    benchmark_float_msg_rti
    benchmark_joint_state_rti
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endif()
