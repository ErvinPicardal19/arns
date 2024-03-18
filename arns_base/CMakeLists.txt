cmake_minimum_required(VERSION 3.8)
project(arns_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(PACKAGE_DEPENDENCIES
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${PACKAGE_DEPENDENCIES})
  find_package(${Dependency} REQUIRED)
endforeach()


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
  ${PACKAGE_DEPENDENCIES}
)

# for adding only locally compiled libraries as dependencies
# target_link_libraries(arns_hardware custom_interfaces)

#-------------------------------------------------

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface arns_hardware.xml)

# INSTALL
install(
  DIRECTORY include
  DESTINATION include/arns_hardware
)

install(TARGETS arns_hardware
  EXPORT export_arns_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_arns_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(${PACKAGE_DEPENDENCIES})

ament_package()
