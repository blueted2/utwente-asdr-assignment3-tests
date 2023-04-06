cmake_minimum_required(VERSION 3.8)
project(closed_loop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
add_compile_options (-Wno-psabi)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(asdfr_interfaces REQUIRED)

add_executable(controller
  src/controller.cpp)
ament_target_dependencies(controller rclcpp std_msgs  asdfr_interfaces)

target_include_directories(controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(controller PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS controller
  DESTINATION lib/${PROJECT_NAME})

ament_package()
