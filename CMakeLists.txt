#
# The find_package macro for Orocos-RTT works best with
# cmake >= 2.6.3
#
cmake_minimum_required(VERSION 2.6.3)

#
# This creates a standard cmake project. You may extend this file with
# any cmake macro you see fit.
#
project(orocos_franka_panda)
# add_definitions(-std=c++11)

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_RELEASE} -Wall")#Generates additional warning messages when compiling

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# catkin_package(
#     CATKIN_DEPENDS etasl_rtt
# )
# Use 'source orocos_toolchain/env.sh' if the command below fails:
find_package(OROCOS-RTT QUIET)
if (NOT OROCOS-RTT_FOUND)
  message (FATAL_ERROR "\nCould not find Orocos. Please use the shell command\n 'source orocos_toolchain/env.sh' and then run cmake again.")
endif()

find_package(catkin REQUIRED
  rtt_motion_control_msgs
  rtt_sensor_msgs
  rtt_trajectory_msgs
  sensor_msgs
  motion_control_msgs
)

# find_package(catkin QUIET OPTIONAL_COMPONENTS rtt_ros roscpp)

# set(CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/libfranka/build:${CMAKE_PREFIX_PATH}")#Generates additional warning messages when compiling

#Comment this when using the binary version of libfranka
set(FRANKA_LIB "${CMAKE_CURRENT_SOURCE_DIR}/libfranka/build/libfranka.so")



# Defines the orocos_* cmake macros. See that file for additional
# documentation.
include( ${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )

# add_subdirectory(libfranka)
# include_directories(/home/haianos/experimental/libfranka/install/include)
# link_directories(/home/haianos/experimental/libfranka/install/lib)

# TODO: FiX me
# link_directories(/home/haianos/frankaws/devel/lib)
# link_directories("${CMAKE_CURRENT_SOURCE_DIR}/libfranka/build")


include_directories(
   libfranka/include
)

# Rest of the configuration is in src/
# add_subdirectory(libfranka)
add_subdirectory(src)
#
# Building a typekit using typegen (recommended):
#
# Creates a typekit library libfranka_panda_driver-types-<target>.so
# and installs in the directory lib/orocos/@target@/franka_panda_driver/types/
#
# The header will go in include/orocos/franka_panda_driver/types/franka_panda_driver/franka_panda_driver-types.hpp
# So you can #include <franka_panda_driver/franka_panda_driver-types.hpp>
#
# orocos_typegen_headers(include/franka_panda_driver/franka_panda_driver-types.hpp) # ...you may add multiple header files
#
# You may only have *ONE* orocos_typegen_headers statement in your toplevel CMakeFile.txt !

# install(TARGETS libfranka
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# )



#
# Generates and installs our package. Must be the last statement such
# that it can pick up all above settings.
#
orocos_generate_package()
