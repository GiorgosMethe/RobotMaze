Project(robotMaze)
cmake_minimum_required( VERSION 2.6 )
set ( CMAKE_BUILD_TYPE Release )

find_package( OpenCV REQUIRED )
find_package(Boost REQUIRED)

include_directories ( ${CMAKE_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR} ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

ADD_EXECUTABLE(
        robotMaze
        main.cpp
        geometry.h
)
 
target_link_libraries( robotMaze ${OpenCV_LIBS} ${Boost_LIBRARIES})
