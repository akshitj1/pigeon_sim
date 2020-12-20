find_package(Eigen3 REQUIRED)
find_package(gazebo REQUIRED)
find_package(Protobuf REQUIRED)

#---------------- message proto files to .pb.h files------------------------
# example ref: https://cmake.org/cmake/help/latest/module/FindProtobuf.html
set(Protobuf_IMPORT_DIRS "")
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(Protobuf_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
    set(GAZEBO_MSG_HEADER_DIR "${ITR}/gazebo/msgs")
  endif()
endforeach()
message(WARN "PROTOBUF_IMPORT_DIRS = " ${Protobuf_IMPORT_DIRS})
message(WARN "GAZEBO_MSG_HEADER_DIR = " ${GAZEBO_MSG_HEADER_DIR})

include_directories(${GAZEBO_INCLUDE_DIRS})
include_directories(${GAZEBO_MSG_HEADER_DIR})

# Get lists of all .proto files in the msgs directory
file(GLOB msgs msgs/*.proto)

# Finally, generate the .cpp files from the .proto files
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${msgs})

# Create a shared library of protobuf messages (.so extension on Linux platforms)
add_library(gazebo_plugin_msgs SHARED ${PROTO_SRCS})
target_link_libraries(gazebo_plugin_msgs ${Protobuf_LIBRARY} gazebo_msgs)
link_libraries(gazebo_plugin_msgs)

#---------------- Gazebo plugins------------------------

include_directories(${Eigen3_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})
add_library(rotors_gazebo_motor_model SHARED gazebo_motor_model.cpp)
target_include_directories(rotors_gazebo_motor_model
  PUBLIC
  ${catkin_INCLUDE_DIRS})
target_link_libraries(rotors_gazebo_motor_model ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Eigen3_LIBRARIES} gazebo_plugin_msgs)
add_dependencies(rotors_gazebo_motor_model ${catkin_EXPORTED_TARGETS})

install(
  TARGETS rotors_gazebo_motor_model gazebo_plugin_msgs
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

