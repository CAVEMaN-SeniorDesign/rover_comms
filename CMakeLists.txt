cmake_minimum_required(VERSION 3.5)
project(rover_comms)

# Use C++20 for compatibility with Ben & Kush's code
set(CMAKE_CXX_STANDARD 20)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)

set(CMAKE_PREFIX_PATH "/usr/local/lib")
set(HEADER_DIR ${CMAKE_PREFIX_PATH}/../include/c++)
set(COMMON_DIR ${CMAKE_PREFIX_PATH}/../include/common)
set(JETSON_CAVE_TALK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Jetson-Comms)
set(CAVETALK_BUILD_DIR ${JETSON_CAVE_TALK_DIR}/CAVeTalk/build) 

set(PROTO_DIR ${CAVETALK_BUILD_DIR}/CAVeTalk-cpp_protos)
set(CAVETALK_DIR ${JETSON_CAVE_TALK_DIR}/CAVeTalk)
set(CAVEHEADER_DIR ${CAVETALK_DIR}/lib/c++/inc)
set(CAVECOMMON_DIR ${CAVETALK_DIR}/lib/common/inc)
set(CATCH_DIR /usr/local/include)

add_subdirectory(${JETSON_CAVE_TALK_DIR})

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED) 
find_package(std_msgs REQUIRED)  
find_package(sensor_msgs REQUIRED) 
find_package(rover_interfaces REQUIRED)
find_package(Catch2 3 REQUIRED)
find_package(serial REQUIRED)
# add_executable(rover_comm src/rover_comm.cpp)
# ament_target_dependencies(rover_comm rclcpp sensor_msgs std_msgs rover_interfaces)

# add_executable(rover_comms_listener src/rover_comms_listener.cpp)
# ament_target_dependencies(rover_comms_listener rclcpp sensor_msgs std_msgs rover_interfaces)

add_executable(rover_comms src/main.cpp src/rover_comm.cpp src/rover_comms_listener.cpp)
ament_target_dependencies(rover_comms rclcpp sensor_msgs std_msgs rover_interfaces)

add_executable(low_level_comms test/low_level_comms.cpp src/rover_comm.cpp src/rover_comms_listener.cpp)
ament_target_dependencies(low_level_comms rclcpp sensor_msgs std_msgs rover_interfaces)

add_executable(high_level_comms test/high_level_comms.cpp src/rover_comm.cpp src/rover_comms_listener.cpp)
ament_target_dependencies(high_level_comms rclcpp sensor_msgs std_msgs rover_interfaces)

add_executable(ben_test test/ben_test.cpp src/rover_comm.cpp src/rover_comms_listener.cpp)
ament_target_dependencies(ben_test rclcpp sensor_msgs std_msgs rover_interfaces)

add_executable(cserial test/cserial.cpp src/rover_comm.cpp src/rover_comms_listener.cpp)
ament_target_dependencies(cserial rclcpp sensor_msgs std_msgs rover_interfaces serial)


set(PROTOBUF_INSTALL_DIR ${JETSON_CAVE_TALK_DIR}/CAVeTalk/external/protobuf/_build/protobuf-install)
add_subdirectory(${CAVETALK_DIR}/external/protobuf/third_party/abseil-cpp)
find_package(Protobuf CONFIG REQUIRED HINTS ${PROTOBUF_INSTALL_DIR})

# add_library(CAVeTalk-cpp ${CAVETALK_DIR}/)

target_include_directories(rover_comms 
    PUBLIC
        ${CAVEHEADER_DIR}
        ${CAVECOMMON_DIR}
        ${JETSON_CAVE_TALK_DIR}/inc
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
        jetson_comms
)

target_include_directories(low_level_comms 
    PUBLIC
        ${CAVEHEADER_DIR}
        ${CAVECOMMON_DIR}
        ${JETSON_CAVE_TALK_DIR}/inc
        ${CAVETALK_BUILD_DIR}
        ${PROTO_DIR}
        ${PROTOBUF_INSTALL_DIR}/include
        ${CATCH_DIR}
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(low_level_comms 
        ${CAVETALK_BUILD_DIR}/libCAVeTalk-cpp.a
        ${CAVETALK_BUILD_DIR}/libCAVeTalk-common.a
        ${CAVETALK_BUILD_DIR}/libCAVeTalk-c.a
        ${CAVETALK_BUILD_DIR}/libCAVeTalk-c_messages.a
        ${CAVETALK_BUILD_DIR}/libCAVeTalk-cpp_messages.a
        protobuf::libprotobuf
        jetson_comms
        Catch2::Catch2WithMain
)

target_include_directories(high_level_comms 
    PUBLIC
        ${CAVEHEADER_DIR}
        ${CAVECOMMON_DIR}
        ${JETSON_CAVE_TALK_DIR}/inc
        ${CAVETALK_BUILD_DIR}
        ${PROTO_DIR}
        ${PROTOBUF_INSTALL_DIR}/include
        ${CATCH_DIR}
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(high_level_comms 
        ${CAVETALK_BUILD_DIR}/libCAVeTalk-cpp.a
        ${CAVETALK_BUILD_DIR}/libCAVeTalk-common.a
        ${CAVETALK_BUILD_DIR}/libCAVeTalk-c.a
        ${CAVETALK_BUILD_DIR}/libCAVeTalk-c_messages.a
        ${CAVETALK_BUILD_DIR}/libCAVeTalk-cpp_messages.a
        protobuf::libprotobuf
        jetson_comms
        Catch2::Catch2WithMain
)

target_include_directories(ben_test 
    PUBLIC
        ${CAVEHEADER_DIR}
        ${CAVECOMMON_DIR}
        ${JETSON_CAVE_TALK_DIR}/inc
        ${CAVETALK_BUILD_DIR}
        ${PROTO_DIR}
        ${PROTOBUF_INSTALL_DIR}/include
        ${CATCH_DIR}
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(ben_test 
        ${CAVETALK_BUILD_DIR}/libCAVeTalk-cpp.a
        ${CAVETALK_BUILD_DIR}/libCAVeTalk-common.a
        ${CAVETALK_BUILD_DIR}/libCAVeTalk-c.a
        ${CAVETALK_BUILD_DIR}/libCAVeTalk-c_messages.a
        ${CAVETALK_BUILD_DIR}/libCAVeTalk-cpp_messages.a
        protobuf::libprotobuf
        jetson_comms
        Catch2::Catch2WithMain
)

target_include_directories(cserial 
    PUBLIC
        ${CAVEHEADER_DIR}
        ${CAVECOMMON_DIR}
        ${JETSON_CAVE_TALK_DIR}/inc
        ${CAVETALK_BUILD_DIR}
        ${PROTO_DIR}
        ${PROTOBUF_INSTALL_DIR}/include
        ${CATCH_DIR}
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(cserial 
        ${CAVETALK_BUILD_DIR}/libCAVeTalk-cpp.a
        ${CAVETALK_BUILD_DIR}/libCAVeTalk-common.a
        ${CAVETALK_BUILD_DIR}/libCAVeTalk-c.a
        ${CAVETALK_BUILD_DIR}/libCAVeTalk-c_messages.a
        ${CAVETALK_BUILD_DIR}/libCAVeTalk-cpp_messages.a
        protobuf::libprotobuf
        jetson_comms
        Catch2::Catch2WithMain
)

# Install
install(
  TARGETS 
    rover_comms
    low_level_comms
    high_level_comms
    ben_test
    cserial
  DESTINATION lib/${PROJECT_NAME}
)

# Build tests if enabled
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
