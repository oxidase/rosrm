# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 3.1)
project(rosrm VERSION 5.12)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FindPkgConfig)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs geometry_msgs genmsg)

## Declare ROS messages and services
add_message_files(FILES
  Annotation.msg
  Approach.msg
  Bearing.msg
  Intersection.msg
  Lane.msg
  Leg.msg
  Maneuver.msg
  Matching.msg
  Overview.msg
  Route.msg
  Step.msg)
add_service_files(FILES
  MatchService.srv
  RouteService.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

## Declare a catkin package
catkin_package()
add_library(catkin INTERFACE IMPORTED)
set_property(TARGET catkin PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${catkin_INCLUDE_DIRS})
set_property(TARGET catkin PROPERTY INTERFACE_LINK_LIBRARIES ${catkin_LIBRARIES})

## Declare libosrm
pkg_check_modules(OSRM REQUIRED libosrm>=5.12)
add_library(libosrm INTERFACE IMPORTED)
set_property(TARGET libosrm PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${OSRM_INCLUDE_DIRS})
set_property(TARGET libosrm PROPERTY INTERFACE_LINK_LIBRARIES ${OSRM_LIBRARIES})

set(CMAKE_EXE_LINKER_FLAGS "-Wl,--unresolved-symbols=ignore-in-shared-libs ${CMAKE_EXE_LINKER_FLAGS}")

## Build service server
add_executable(rosrm_server src/rosrm_server.cpp)
target_link_libraries(rosrm_server catkin libosrm)
add_dependencies(rosrm_server rosrm_gencpp)

# %EndTag(FULLTEXT)%
