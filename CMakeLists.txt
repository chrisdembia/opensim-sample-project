cmake_minimum_required(VERSION 2.8)
project(opensim-sample-client)

find_package(OpenSim REQUIRED)

if ("${CMAKE_CXX_COMPILER_ID}" MATCHES "GNU" OR
        "${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif()

include_directories(${OpenSim_INCLUDE_DIRS})
link_directories(${OpenSim_LIB_DIR})

add_executable(myexe main.cpp)
target_link_libraries(myexe ${OpenSim_LIBRARIES})
