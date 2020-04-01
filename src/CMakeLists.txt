
#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#

# Creates a component library libfranka_panda_driver-<target>.so
# and installs in the directory lib/orocos/franka_panda_driver/
#
orocos_component(FrankaComponent franka_panda_driver-component.hpp franka_panda_driver-component.cpp) # ...you may add multiple source files

# target_link_libraries(FrankaComponent /home/santiregui/etasl-install/ws/etasl-ex/devel/lib/lib/libfranka.so.0.7.1)

# find_library(FRANKA_LIB franka HINTS /lib/)
find_library(FRANKA_LIB franka)
target_link_libraries(FrankaComponent ${FRANKA_LIB})




include_directories(
   # include
   ${catkin_INCLUDE_DIRS}
)

#
# You may add multiple orocos_component statements.

#
# Additional headers:
#
# Installs in the include/orocos/franka_panda_driver/ directory
#
# orocos_install_headers(franka_panda_driver-component.hpp) # ...you may add multiple header files
#
# You may add multiple orocos_install_headers statements.

#
# Building a Plugin
#
# Creates a plugin library libfranka_panda_driver-plugin-<target>.so
# and installs in the directory lib/orocos/franka_panda_driver/plugins/
#
# Be aware that a plugin may only have the loadRTTPlugin() function once defined in a .cpp file.
# This function is defined by the plugin and service CPP macros.
#
# orocos_plugin(franka_panda_driver-plugin franka_panda_driver-plugin.cpp) # ...only one plugin function per library !
#
# You may add multiple orocos_plugin statements.


#
# Building a Service:
#
# Creates a plugin library libfranka_panda_driver-service-<target>.so
# and installs in the directory lib/orocos/franka_panda_driver/plugins/
#
# orocos_service(franka_panda_driver-service franka_panda_driver-service.cpp) # ...only one service per library !
#
# You may add multiple orocos_service statements.


#
# Building a normal library (optional):
#
# Creates a library libfranka_panda_driver-support-<target>.so and installs it in
# lib/
#
# orocos_library(franka_panda_driver-support support.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_library statements.
