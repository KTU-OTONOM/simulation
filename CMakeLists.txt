cmake_minimum_required(VERSION 3.5)
project(simulation)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

include_directories(include)
include_directories(simulation)

# Tek satırda gerekli klasörleri yükle
install(DIRECTORY launch worlds scripts meshes urdf models
  DESTINATION share/${PROJECT_NAME}
)

# Python paketi değilse, bu satırı yorumlayabilirsin
# ament_python_install_package(${PROJECT_NAME})

ament_package()

