cmake_minimum_required(VERSION 3.3)
project(boids)

set(CMAKE_CXX_STANDARD_REQUIRED 11)

find_package(
    OpenCV REQUIRED
)

include_directories(
    include/
    ${OpenCV_INCLUDE_DIRS}
)

set(SOURCES
    src/main.cpp
)

set(HEADERS
)

set(CMAKE_BUILD_TYPE Debug) 
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")  

add_executable( main ${SOURCES} ${HEADERS} )
target_link_libraries( main ${OpenCV_LIBS} )