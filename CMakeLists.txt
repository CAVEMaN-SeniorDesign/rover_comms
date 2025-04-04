cmake_minimum_required(VERSION 3.5)
project(rover_comms)

# Use C++20 for compatibility with Ben & Kush's code
# 
# NOTE: NEED TO CLEANUP AND MAKE SHARED LIB INSTEAD OF COPY PASTE OVER AND OVER 
# 

set(CMAKE_CXX_STANDARD 20)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -fdiagnostics-color=always)
endif()

include_directories(include)

set(CMAKE_PREFIX_PATH "/usr/local/lib")
set(HEADER_DIR ${CMAKE_PREFIX_PATH}/../include/c++)
set(COMMON_DIR ${CMAKE_PREFIX_PATH}/../include/common)
set(CAVE_TALK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external/CAVeTalk)
set(CAVETALK_BUILD_DIR ${CAVE_TALK_DIR}/build) 

set(PROTO_DIR ${CAVETALK_BUILD_DIR}/CAVeTalk-cpp_protos)
set(CAVETALK_DIR ${CAVE_TALK_DIR})
set(CAVEHEADER_DIR ${CAVETALK_DIR}/lib/c++/inc)
set(CAVECOMMON_DIR ${CAVETALK_DIR}/lib/common/inc)
set(CATCH_DIR /usr/local/include)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED) 
find_package(std_msgs REQUIRED)  
find_package(sensor_msgs REQUIRED) 
find_package(rover_interfaces REQUIRED)
find_package(Catch2 3 REQUIRED)
find_package(serial REQUIRED)

add_library(rover_comms_serial STATIC src/rover_comms_serial.cpp)
ament_target_dependencies(rover_comms_serial rclcpp std_msgs serial ament_index_cpp)

target_include_directories(rover_comms_serial
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CAVEHEADER_DIR}
        ${CAVECOMMON_DIR}
        ${CAVE_TALK_DIR}/inc
        ${CAVETALK_BUILD_DIR}
        ${PROTO_DIR}
        ${PROTOBUF_INSTALL_DIR}/include
        external/tinyxml/inc
)

target_link_libraries(rover_comms_serial
        ${CAVETALK_BUILD_DIR}/libCAVeTalk-cpp.a
        ${CAVETALK_BUILD_DIR}/libCAVeTalk-common.a
        ${CAVETALK_BUILD_DIR}/libCAVeTalk-c.a
        ${CAVETALK_BUILD_DIR}/libCAVeTalk-c_messages.a
        ${CAVETALK_BUILD_DIR}/libCAVeTalk-cpp_messages.a
        protobuf::libprotobuf
)

add_executable(rover_comms src/main.cpp src/rover_comm.cpp src/rover_comms_listener.cpp external/tinyxml/src/tinyxml2.cpp)
ament_target_dependencies(rover_comms rclcpp sensor_msgs std_msgs rover_interfaces serial)

set(PROTOBUF_INSTALL_DIR ${CAVE_TALK_DIR}/external/protobuf/_build/protobuf-install)
add_subdirectory(${CAVETALK_DIR}/external/protobuf/third_party/abseil-cpp)

find_package(utf8_range CONFIG REQUIRED HINTS ${PROTOBUF_INSTALL_DIR})
find_package(absl CONFIG REQUIRED HINTS ${PROTOBUF_INSTALL_DIR})
find_package(Protobuf CONFIG REQUIRED HINTS ${PROTOBUF_INSTALL_DIR})

target_include_directories(rover_comms 
    PUBLIC
        ${CAVEHEADER_DIR}
        ${CAVECOMMON_DIR}
        ${CAVE_TALK_DIR}/inc
        ${CAVETALK_BUILD_DIR}
        ${PROTO_DIR}
        ${PROTOBUF_INSTALL_DIR}/include
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(rover_comms 
        ${CAVETALK_BUILD_DIR}/libCAVeTalk-cpp.a
        ${CAVETALK_BUILD_DIR}/libCAVeTalk-common.a
        ${CAVETALK_BUILD_DIR}/libCAVeTalk-c.a
        ${CAVETALK_BUILD_DIR}/libCAVeTalk-c_messages.a
        ${CAVETALK_BUILD_DIR}/libCAVeTalk-cpp_messages.a
        protobuf::libprotobuf
        ament_index_cpp::ament_index_cpp
        rover_comms_serial
)

# Install
install(
  TARGETS 
    rover_comms
    rover_comms_serial
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY configs
  DESTINATION share/${PROJECT_NAME}
)

# Build tests if enabled
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
