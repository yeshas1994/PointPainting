cmake_minimum_required(VERSION 3.1 FATAL_ERROR)

project(PointPainting)

set(Torch_DIR include/libtorch/share/cmake/Torch)
set(Caffe2_DIR include/libtorch/share/cmake/Caffe2)
  
find_package(Torch REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

set (PATH ${CMAKE_CURRENT_SOURCE_DIR}/include/govtech)
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

include_directories(
  	include 
  	${PCL_INCLUDE_DIRS}
  	${EIGEN3_INCLUDE_DIRS}
  	${OpenCV_INCLUDE_DIRS}
  	${PATH}/websocketpp/
	${PATH}/asio-1.12.2/include/
	${PATH}/msgpack11/
	${PATH}/stb/
	${PATH}/dosFramework/
	${PATH}/json/single_include/
	${PCL_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
)


link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_definitions(${EIGEN3_DEFINITIONS})

add_executable (infer_server inference_server.cpp)
target_link_libraries (infer_server 
	${PCL_LIBRARIES} 
	${TORCH_LIBRARIES}
	${OpenCV_LIBS} 
	Eigen3::Eigen
	GOVTECH_LIB
)

set_property(TARGET infer_server PROPERTY CXX_STANDARD 14)