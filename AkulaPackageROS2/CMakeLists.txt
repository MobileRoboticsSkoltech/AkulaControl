cmake_minimum_required(VERSION 3.5)
project(akula_package)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#---ROS packages---#
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(akula_package REQUIRED)
#---ROS packages---#

#---YAML---#
find_package(yaml-cpp)
#---YAML---#

#---Executables---#
add_executable(AkulaMainNode
        src/AkulaMainNode.cpp
        src/SerialConnector.cpp
        src/dSocket/dSocket.cpp)
add_executable(AkulaEncoderNode
        src/AkulaEncoderNode.cpp
        src/dSocket/dSocket.cpp)
#---Executables---#

#---Include---#
target_include_directories(AkulaMainNode PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(AkulaEncoderNode PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
#---Include---#

target_link_libraries(AkulaMainNode
        ${YAML_CPP_LIBRARIES})

#---Ament---#
ament_target_dependencies(AkulaMainNode
        rclcpp
        std_msgs
        sensor_msgs
        ament_index_cpp
        akula_package)
ament_target_dependencies(AkulaEncoderNode
        rclcpp
        ament_index_cpp
        akula_package)
#---Ament---#

rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/Encoders.msg")

install(TARGETS AkulaMainNode AkulaEncoderNode
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY launch model config
        DESTINATION share/${PROJECT_NAME}/
)

ament_package()
