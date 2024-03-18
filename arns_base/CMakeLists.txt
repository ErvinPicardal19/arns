cmake_minimum_required(VERSION 3.8)
project(arns_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_srvs REQUIRED)

#-------------------------------------------------
# BUILDING THE LIBRARY

# handles exporting arns_system libraries and headers
add_library(arns_hardware SHARED 
  src/arns_hardware.cpp
  src/arduino_comms.cpp
  src/wheel.cpp
)

# indicate include directory for the target
target_include_directories(arns_hardware PRIVATE include) 

# does same thing above
# include_directories(include)

#-------------------------------------------------
# LINKING DEPENDENCIES

# for adding non-locally or locally compiled libraries as dependencies (e.g. find_package)
ament_target_dependencies(
  arns_hardware PUBLIC
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# for adding only locally compiled libraries as dependencies
# target_link_libraries(arns_hardware custom_interfaces)
target_link_libraries(arns_hardware PUBLIC serial)

#-------------------------------------------------

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface arns_hardware.xml)

# INSTALL
install(
  DIRECTORY include
  DESTINATION include/arns_hardware
)

install(
  TARGETS arns_hardware
  DESTINATION share/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

include_directories(include)


ament_export_libraries(arns_hardware)

ament_package()
