cmake_minimum_required(VERSION 3.17)
project(DesktopServer)

set(CMAKE_CXX_STANDARD 17)

#set (CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fsanitize=undefined -fsanitize=address")
#set (CMAKE_LINKER_FLAGS_DEBUG "${CMAKE_LINKER_FLAGS_DEBUG} -fsanitize=undefined -fsanitize=address")

#---Threads---#
if (UNIX AND NOT APPLE)
    set(THREADS_PREFER_PTHREAD_FLAG ON)
    find_package(Threads REQUIRED)
endif()
#---Threads---#

#---YAML---#
find_package(yaml-cpp)
#---YAML---#

include_directories(lib)

add_executable(DesktopServer
        main.cpp
        Server.cpp
        SerialConnector.cpp
        SerialMonitor.cpp
        lib/dSocket/dSocket.cpp MotorPWM.cpp MotorPWM.h)
target_link_libraries(DesktopServer
        Threads::Threads
        ${YAML_CPP_LIBRARIES})