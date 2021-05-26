cmake_minimum_required(VERSION 3.19)
project(SerialConnector)

set(CMAKE_CXX_STANDARD 17)

#---Threads---#
if (UNIX AND NOT APPLE)
    set(THREADS_PREFER_PTHREAD_FLAG ON)
    find_package(Threads REQUIRED)
endif()
#---Threads---#

add_executable(SerialConnector
        main.cpp
        SerialConnector.cpp
        SerialMonitor.cpp)
target_link_libraries(SerialConnector
        Threads::Threads)