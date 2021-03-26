cmake_minimum_required(VERSION 3.17)
project(DesktopServer)

set(CMAKE_CXX_STANDARD 17)

#---Threads---#
if (UNIX AND NOT APPLE)
    set(THREADS_PREFER_PTHREAD_FLAG ON)
    find_package(Threads REQUIRED)
endif()
#---Threads---#

include_directories(lib)

add_executable(DesktopServer
        main.cpp
        Server.cpp
        lib/dSocket/dSocket.cpp)
target_link_libraries(DesktopServer
        Threads::Threads)