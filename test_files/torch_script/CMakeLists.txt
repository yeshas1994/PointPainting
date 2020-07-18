cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(test)
set(CMAKE_PREFIX_PATH=/home/yeshas/ARC/PointPainting/torch_script/libtorch/)
  
find_package(Torch REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
   ${OpenCV_INCLUDE_DIRS}
)


add_executable(example test.cpp)
target_link_libraries(example ${TORCH_LIBRARIES} ${OpenCV_LIBS})
set_property(TARGET example PROPERTY CXX_STANDARD 14)
