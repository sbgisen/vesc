cmake_minimum_required(VERSION 3.8)
project(vesc_hw_interface)

add_compile_options(-std=c++17)

if(CMAKE_COMPILER_IS_GNUCXX
   OR CMAKE_CXX_COMPILER_ID
      MATCHES
      "Clang"
)
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
  )
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(Threads)
ament_auto_find_build_dependencies()

ament_auto_add_library(
  ${PROJECT_NAME}
  SHARED
  src/vesc_hw_interface.cpp
  src/vesc_servo_controller.cpp
  src/vesc_wheel_controller.cpp
  src/vesc_step_difference.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC include)

ament_target_dependencies(${PROJECT_NAME} ${DEPENDENCIES})
target_compile_definitions(${PROJECT_NAME} PRIVATE "HARDWARE_INTERFACE_BUILDING_DLL")

pluginlib_export_plugin_description_file(hardware_interface vesc_hw_interface_plugin.xml)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
