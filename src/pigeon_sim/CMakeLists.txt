cmake_minimum_required(VERSION 3.5.1)
project(pigeon_sim)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_DEBUG_INIT "-Wall")
set(CMAKE_CXX_FLAGS "-Wno-dev")
set(CMAKE_BUILD_TYPE Debug)
# set(CMAKE_MESSAGE_LOG_LEVEL STATUS)

find_package(catkin REQUIRED 
  COMPONENTS 
  roscpp
  gazebo_ros
  gazebo_msgs
  mav_msgs)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
find_package(TinyXML2 8.0.0 MODULE REQUIRED)

find_package(ignition-math4 REQUIRED)

catkin_package(
  DEPENDS
  TinyXML2
)

add_subdirectory(src/gazebo_plugins)

add_executable(tailsitter_spawner_node
  src/tailsitter_spawner_node.cpp 
  src/tailsitter_spawner.hpp
  src/gazebo_client.hpp)

target_include_directories(
  tailsitter_spawner_node 
  PUBLIC 
  ${TinyXML2_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS})

target_link_libraries(
  tailsitter_spawner_node
  ignition-math4::ignition-math4
  ${TinyXML2_LIBRARIES}
  ${catkin_LIBRARIES})

install(TARGETS tailsitter_spawner_node
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY launch worlds meshes DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
