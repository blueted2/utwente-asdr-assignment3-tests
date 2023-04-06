cmake_minimum_required(VERSION 3.8)
project(light_pos)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_options(-Wno-psabi)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(asdfr_interfaces REQUIRED)

add_executable(light_pos
  src/light_pos.cpp
  src/image_functions.cpp
  )
  
ament_target_dependencies(light_pos rclcpp std_msgs sensor_msgs asdfr_interfaces)
target_include_directories(light_pos PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
  
target_compile_features(light_pos PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS light_pos
  DESTINATION lib/${PROJECT_NAME})

#  Install launch directory
install (DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
  )

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_package()
