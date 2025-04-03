cmake_minimum_required(VERSION 3.8)
project(ur10_rl_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# add subdirectories
add_subdirectory(src/finite_state_machine)
add_subdirectory(src/robot)


ament_package()
