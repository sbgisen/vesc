cmake_minimum_required(VERSION 3.8)
project(vesc_driver)

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

find_package(ament_cmake_auto REQUIRED)
find_package(Threads)
ament_auto_find_build_dependencies()

###########
## Build ##
###########

# driver libraries
ament_auto_add_library(
  ${PROJECT_NAME}
  SHARED
  src/vesc_interface.cpp
  src/vesc_packet.cpp
  src/vesc_packet_factory.cpp
)

ament_auto_package()
