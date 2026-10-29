cmake_minimum_required(VERSION 3.16)

find_package(yaml-cpp REQUIRED)

add_subdirectory(serial)

# Independent ASCII serial interface for NUC-to-gimbal motion and CSV feedback.
# This target intentionally does not link the legacy io library, CBoard, or
# the binary Gimbal protocol.
add_library(gimbal1 STATIC gimbal1/gimbal1.cpp)
target_include_directories(gimbal1 PUBLIC ${PROJECT_SOURCE_DIR})
target_link_libraries(gimbal1 PUBLIC fmt::fmt yaml-cpp serial)

set(DAHENG_SDK_ROOT
  "${PROJECT_SOURCE_DIR}/third_party/daheng_sdk/sdk_payload/Galaxy_camera"
  CACHE PATH "Daheng Galaxy SDK root directory")

# Drop a stale cached include path when an SDK folder was moved or removed.
if(DAHENG_INCLUDE_DIR AND NOT EXISTS "${DAHENG_INCLUDE_DIR}/GxIAPI.h")
  unset(DAHENG_INCLUDE_DIR CACHE)
endif()

find_path(DAHENG_INCLUDE_DIR
  NAMES GxIAPI.h
  PATHS
    "${DAHENG_SDK_ROOT}/inc"
    "/usr/include"
    "/usr/local/include"
  NO_DEFAULT_PATH)

if(NOT DAHENG_INCLUDE_DIR)
  message(FATAL_ERROR "Daheng Galaxy SDK header GxIAPI.h not found. Set -DDAHENG_SDK_ROOT=/path/to/Galaxy_camera")
endif()

# 创建目标 io
add_library(io STATIC 
    hikrobot/hikrobot.cpp    
    mindvision/mindvision.cpp  
    usbcamera/usbcamera.cpp  
    daheng/daheng.cpp
    camera.cpp
    cboard.cpp
    dm_imu/dm_imu.cpp
    gimbal/gimbal.cpp
)

# hikrobot
target_include_directories(io PUBLIC hikrobot/include)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  target_link_directories(io PUBLIC hikrobot/lib/amd64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  target_link_directories(io PUBLIC hikrobot/lib/arm64)
else()
  message(FATAL_ERROR "Unsupported architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}!")
endif()

# mindvision
target_include_directories(io PUBLIC mindvision/include)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  target_link_directories(io PUBLIC mindvision/lib/amd64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  target_link_directories(io PUBLIC mindvision/lib/arm64)
else()
  message(FATAL_ERROR "Unsupported architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}!")
endif()

target_include_directories(io PUBLIC ${DAHENG_INCLUDE_DIR})
target_link_libraries(io MvCameraControl MVSDK gxiapi usb-1.0 yaml-cpp serial)

################## 检查 ROS 环境 ##################
find_package(ament_cmake QUIET)
find_package(rclcpp QUIET)
find_package(std_msgs QUIET)
find_package(rosidl_typesupport_cpp QUIET)
find_package(sp_msgs QUIET)

# 如果找不到 ROS 环境，则跳过 ROS 相关的部分
if(NOT ament_cmake_FOUND OR NOT rclcpp_FOUND OR NOT std_msgs_FOUND OR NOT rosidl_typesupport_cpp_FOUND OR NOT sp_msgs_FOUND)
    message(WARNING "ROS2 not found, skipping ROS2 specific code.")
else()
    message(STATUS "ROS2 environment found, compiling ROS2-related code.")
    
    # 添加 ROS2 相关的文件
    target_sources(io PRIVATE ros2/publish2nav.cpp ros2/ros2.cpp ros2/subscribe2nav.cpp)

    # 设置 ROS2 依赖
    include_directories(${std_msgs_INCLUDE_DIRS})
    ament_target_dependencies(io rclcpp std_msgs sp_msgs)
endif()
