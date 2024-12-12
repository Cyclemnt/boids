cmake_minimum_required(VERSION 3.3)
project(boids)

set(CMAKE_CXX_STANDARD_REQUIRED 17)

find_package(
    OpenCV REQUIRED
)
find_package(OpenMP REQUIRED)

include_directories(
    include/
    ${OpenCV_INCLUDE_DIRS}
)

set(SOURCES
    src/main.cpp
    src/boid.cpp
    src/simulation.cpp
    src/zone.cpp
)

set(HEADERS
    include/types.hpp
    include/boid.hpp
    include/simulation.hpp
    include/zone.hpp
)

set(CMAKE_BUILD_TYPE Debug) 
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")  

add_executable( main ${SOURCES} ${HEADERS} )
target_link_libraries( main OpenMP::OpenMP_CXX ${OpenCV_LIBS} )