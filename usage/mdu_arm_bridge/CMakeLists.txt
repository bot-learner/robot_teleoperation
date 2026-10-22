cmake_minimum_required(VERSION 3.24)
project(a3_mdu_arm_bridge LANGUAGES CXX)
include(FetchContent)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# 仅使用与固件匹配的 AimRT 1.7.0 头文件，不编译/替换机器人运行库。
set(AIMRT_SOURCE_DIR "" CACHE PATH "Existing AimRT 1.7.0 source tree")
if(NOT AIMRT_SOURCE_DIR)
  include(FetchContent)
  FetchContent_Declare(aimrt_headers
    URL https://codeload.github.com/AimRT/AimRT/tar.gz/refs/tags/v1.7.0
    URL_HASH SHA256=257628359f3c8dff3076290389b41adb239b3d9d807bdfebbf8fe21e6aa8e4a7)
  FetchContent_Populate(aimrt_headers)
  set(AIMRT_SOURCE_DIR "${aimrt_headers_SOURCE_DIR}")
endif()
set(UNIFEX_SOURCE_DIR "" CACHE PATH "Existing libunifex source tree")
if(NOT UNIFEX_SOURCE_DIR)
  FetchContent_Declare(unifex_headers
    URL https://codeload.github.com/facebookexperimental/libunifex/tar.gz/591ec09e7d51858ad05be979d4034574215f5971
    URL_HASH SHA256=292d1d383b734de6e926311bd847a0ac18a5dbbbe374c04d29619aeb05fc8651)
  FetchContent_Populate(unifex_headers)
  set(UNIFEX_SOURCE_DIR "${unifex_headers_SOURCE_DIR}")
endif()
set(UNIFEX_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
# 复用 sonic_for_a3 中已有的两项修复，只改本次构建的头文件副本。
find_program(PATCH_EXECUTABLE patch REQUIRED)
set(PATCHED_INTERFACE "${CMAKE_CURRENT_BINARY_DIR}/patched_interface")
file(COPY "${AIMRT_SOURCE_DIR}/src/interface/aimrt_module_ros2_interface" DESTINATION "${PATCHED_INTERFACE}")
configure_file("${AIMRT_SOURCE_DIR}/src/interface/aimrt_module_ros2_interface/util/ros2_type_support.h"
  "${PATCHED_INTERFACE}/aimrt_module_ros2_interface/util/ros2_type_support.h" COPYONLY)
foreach(fix typesupport serialization)
  execute_process(COMMAND "${PATCH_EXECUTABLE}" --batch -p3 -i "${CMAKE_CURRENT_SOURCE_DIR}/patches/${fix}.patch"
    WORKING_DIRECTORY "${PATCHED_INTERFACE}" RESULT_VARIABLE patch_result)
  if(NOT patch_result EQUAL 0)
    message(FATAL_ERROR "Failed to apply ${fix} compatibility fix")
  endif()
endforeach()
add_subdirectory("${UNIFEX_SOURCE_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/unifex" EXCLUDE_FROM_ALL)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(joint_msgs REQUIRED)
find_package(fmt REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(jsoncpp REQUIRED)
add_library(a3_mdu_arm_bridge SHARED bridge.cpp)
target_compile_definitions(a3_mdu_arm_bridge PRIVATE AIMRT_USE_FMT_LIB)
target_include_directories(a3_mdu_arm_bridge PRIVATE
  "${PATCHED_INTERFACE}"
  "${AIMRT_SOURCE_DIR}/src/interface" "${AIMRT_SOURCE_DIR}/src/common"
  "${AIMRT_SOURCE_DIR}/src/common/ros2_util" "${UNIFEX_SOURCE_DIR}/include")
ament_target_dependencies(a3_mdu_arm_bridge rclcpp sensor_msgs std_msgs joint_msgs)
target_link_libraries(a3_mdu_arm_bridge fmt::fmt yaml-cpp JsonCpp::JsonCpp unifex)
