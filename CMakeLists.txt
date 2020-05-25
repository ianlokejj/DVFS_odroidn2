cmake_minimum_required(VERSION 2.8)
project( DisplayImage )
find_package( OpenCV REQUIRED )
find_package( Threads REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
add_executable( DisplayImage types.cpp object_detection.cpp )
target_link_libraries( DisplayImage ${OpenCV_LIBS} ${CMAKE_THREAD_LIBS_INIT} )

