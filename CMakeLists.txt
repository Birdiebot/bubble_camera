cmake_minimum_required(VERSION 3.5)
project(bubble_camera)

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


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)

set(DEPENDENCIES
  rclcpp
  sensor_msgs 
  cv_bridge
  image_transport
  rclcpp_components
)

set(SOURCES
  src/rawImg_process.cpp
  src/vInput_base_node.cpp
  src/vInput_cam.cpp
  src/gx_camera.cpp
)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

add_library(${PROJECT_NAME} SHARED
    include/bubble_camera/camera.hpp
    include/bubble_camera/gxapi/DxImageProc.h
    include/bubble_camera/gxapi/GxIAPI.h
    include/bubble_camera/gxapi/gx_utils.h
    ${SOURCES}
)

target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBRARIES}
    gxiapi
    )

  
ament_target_dependencies(${PROJECT_NAME}
  ${DEPENDENCIES}
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "vInput"
  EXECUTABLE cam
)

# Install binaries
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include
  DESTINATION include
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}/
)

# Ament exports
ament_export_include_directories(include/bubble_camera)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${DEPENDENCIES})

ament_package()