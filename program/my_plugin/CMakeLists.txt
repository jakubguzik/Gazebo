cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# find gazebo
include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo REQUIRED)
else()
  message(FATAL_ERROR "pkg-config is required; please install it")
endif()
include_directories(${GAZEBO_INCLUDE_DIRS} inc)
link_directories(${GAZEBO_LIBRARY_DIRS})

# compile plugin
add_library(my_plugin SHARED my_plugin.cc PID.cpp)
target_link_libraries(my_plugin ${GAZEBO_LIBRARIES})

