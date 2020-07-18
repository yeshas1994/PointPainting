cmake_minimum_required(VERSION 3.10.2)
project(segmentation)

add_compile_options(-std=c++11)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
	roscpp 
	std_msgs  
  image_transport
  cv_bridge
	sensor_msgs
	tf	
)

set(Caffe2_DIR "include/libtorch/share/cmake/Caffe2")
set(Torch_DIR "include/libtorch/share/cmake/Torch")

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Torch REQUIRED)

catkin_package()

## Include files
include_directories(
  include 
  ${catkin_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_definitions(${EIGEN3_DEFINITIONS})

add_executable(segmentation_node
  node/torch_segment_node.cpp
  src/torch_segment.cpp
)

target_link_libraries(segmentation_node
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES} 
  ${TORCH_LIBRARIES}
)
set_property(TARGET segmentation_node PROPERTY CXX_STANDARD 14)
