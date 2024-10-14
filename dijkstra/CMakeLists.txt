cmake_minimum_required(VERSION 3.11)
project(dijkstra)

set(CMAKE_CXX_FLAGS "-std=c++17 -O2 -pthread")
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_MODULE_PATH 
    ${CMAKE_MODULE_PATH} 
    ${CMAKE_CURRENT_SOURCE_DIR}
)
find_package(Eigen3 REQUIRED)
find_package(OpenCV 4 REQUIRED)

set(INCLUDE_DIRS ${INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR})
set(SRC ${SRC})

include_directories(${INCLUDE_DIRS})

add_executable (dijkstra main.cc ${SRC})
target_link_libraries(dijkstra PUBLIC ${OpenCV_LIBS})
target_include_directories(dijkstra PUBLIC ${OpenCV_LIBS} ${CMAKE_SOURCE_DIR})