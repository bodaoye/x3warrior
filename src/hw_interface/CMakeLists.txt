cmake_minimum_required(VERSION 3.5)
project(hw_interface)

# Default to C99
if(NOT CMAKE_C_STANDRD)
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
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

link_directories (${PROJECT_SOURCE_DIR}/lib)


add_library(
  ${PROJECT_NAME}
  SHARED
  src/warrior_interface.cpp
  src/mecanumbot_serial_port.cpp
)

target_link_libraries (${PROJECT_NAME}  libcontrolcan_x86.so)
target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)



ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
)

pluginlib_export_plugin_description_file(hardware_interface warrior_hardware.xml)

install(
  TARGETS
    ${PROJECT_NAME}
  DESTINATION lib
)
install(
  DIRECTORY
    include
  DESTINATION include
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
)

ament_package()
