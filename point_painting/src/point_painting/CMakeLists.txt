cmake_minimum_required(VERSION 3.10.2)
project(point_painting)

add_compile_options(-std=c++14)


## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
	roscpp 
	std_msgs  
  image_transport
  cv_bridge
	sensor_msgs
	tf	
  message_filters
  nav_msgs
)

set(Caffe2_DIR "include/libtorch/share/cmake/Caffe2")
set(Torch_DIR "include/libtorch/share/cmake/Torch")

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Torch REQUIRED)
## Declare ROS messages and services

## Generate added messages and services

## Declare a catkin package
catkin_package()

set (PATH ${CMAKE_CURRENT_SOURCE_DIR}/include/govtech)

## Source files
set(TARGET_SRC
	${PATH}/dosFramework/dosCore/wsServer.cpp
	${PATH}/dosFramework/dosCore/wsServer.h
	${PATH}/dosFramework/dosCore/wsClient.cpp
    ${PATH}/dosFramework/dosCore/wsClient.h
    ${PATH}/dosFramework/dosCore/wsRobotClient.h
    ${PATH}/dosFramework/dosCore/wsRobotClient.cpp
    ${PATH}/dosFramework/dosCore/commandPacket.h
    ${PATH}/dosFramework/dosCore/commandPacket.cpp
    ${PATH}/dosFramework/dosCore/commandPacketParse.h
    ${PATH}/dosFramework/dosCore/commandPacketParse.cpp
    ${PATH}/msgpack11/msgpack11.cpp
    ${PATH}/msgpack11/msgpack11.hpp
    ${PATH}/json/single_include/nlohmann/json.hpp
    ${PATH}/stb/stb_image.h	
)

add_library(GOVTECH_LIB ${TARGET_SRC})

## Include files
include_directories(
  include 
  ${catkin_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PATH}/websocketpp/
	${PATH}/asio-1.12.2/include/
	${PATH}/msgpack11/
	${PATH}/stb/
	${PATH}/dosFramework/
	${PATH}/json/single_include/
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_definitions(${EIGEN3_DEFINITIONS})

add_executable(point_painting_node
  node/point_painting_node.cpp
  src/point_painting.cpp
)

add_executable(segmentation_node
  node/torch_segment_node.cpp
  src/torch_segment.cpp
)

target_link_libraries(segmentation_node
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES} 
  ${TORCH_LIBRARIES}
  ${PCL_LIBRARIES} 
  GOVTECH_LIB
)

target_link_libraries(point_painting_node
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES} 
  ${TORCH_LIBRARIES}
  ${PCL_LIBRARIES} 
  GOVTECH_LIB
)


set_property(TARGET point_painting_node PROPERTY CXX_STANDARD 14)
set_property(TARGET segmentation_node PROPERTY CXX_STANDARD 14)

