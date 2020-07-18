cmake_minimum_required(VERSION 3.1 FATAL_ERROR)

project(PointPainting)
set(CMAKE_CXX_STANDARD 14)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

message("Opencv Library is at ${OpenCV}")
message("OpenCV_LIB at: ${OpenCV_LIBS}")
message("OpenCV_LIBRARIES at: ${OpenCV_LIBRARIES}")
message("OpenCV Include at: ${OpenCV_INCLUDE_DIRS}")

include_directories(
	${PCL_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
)


link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (PointPainting main.cpp)
target_link_libraries (PointPainting 
	${PCL_LIBRARIES} 
	${OpenCV_LIBS} 
	Eigen3::Eigen
)
