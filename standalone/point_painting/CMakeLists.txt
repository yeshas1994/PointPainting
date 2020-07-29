cmake_minimum_required(VERSION 3.1 FATAL_ERROR)

project(PointPainting)

set(Torch_DIR include/libtorch/share/cmake/Torch)
set(Caffe2_DIR include/libtorch/share/cmake/Caffe2)
  
find_package(Torch REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

message("Opencv Library is at ${OpenCV}")
message("OpenCV_LIB at: ${OpenCV_LIBS}")
message("OpenCV_LIBRARIES at: ${OpenCV_LIBRARIES}")
message("OpenCV Include at: ${OpenCV_INCLUDE_DIRS}")

include_directories(
	include
	${PCL_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
)


link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_definitions(${EIGEN3_DEFINITIONS})

add_executable (PointPainting main.cpp)
target_link_libraries (PointPainting 
	${PCL_LIBRARIES} 
	${TORCH_LIBRARIES}
	${OpenCV_LIBS} 
	Eigen3::Eigen
)

set_property(TARGET PointPainting PROPERTY CXX_STANDARD 14)