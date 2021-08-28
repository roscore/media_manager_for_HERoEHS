cmake_minimum_required(VERSION 2.8.3)
project(media_manager)

#add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
)

find_package(PkgConfig)
pkg_check_modules(GSTREAMER QUIET libgstreamer-0.10)

if(NOT GSTREAMER_FOUND)
  set(GSTREAMER_VERSION_1_x TRUE)
endif()

if(GSTREAMER_VERSION_1_x)
  message(STATUS "gst 1.0")
  pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
  pkg_check_modules(GST_APP REQUIRED gstreamer-app-1.0)
else()
  message(STATUS "gst 0.1")
  pkg_check_modules(GSTREAMER REQUIRED gstreamer-0.10)
  pkg_check_modules(GST_APP REQUIRED gstreamer-app-0.10)
endif()

pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR
  NAMES yaml_cpp.h
  PATHS ${YAML_CPP_INCLUDE_DIRS}
)
find_library(YAML_CPP_LIBRARY
  NAMES YAML_CPP
  PATHS ${YAML_CPP_LIBRARY_DIRS}
)
link_directories(${YAML_CPP_LIBRARY_DIRS})

if(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")
add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")

catkin_package(
  INCLUDE_DIRS include
 
  CATKIN_DEPENDS roscpp roslib std_msgs 
  #DEPENDS system_lib
  DEPENDS GSTREAMER

)

include_directories(include ${catkin_INCLUDE_DIRS} ${GSTREAMER_INCLUDE_DIRS})

 add_executable(media_manager_node src/media_manager.cpp)
 target_link_libraries(media_manager_node ${catkin_LIBRARIES} ${GSTREAMER_LIBRARIES} ${YAML_CPP_LIBRARIES})
