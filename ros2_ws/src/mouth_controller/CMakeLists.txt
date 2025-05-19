cmake_minimum_required(VERSION 3.8)
project(mouth_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencias ROS
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# PortAudio y FFTW
find_package(PkgConfig REQUIRED)
pkg_check_modules(PORTAUDIO REQUIRED portaudio-2.0)

# Ejecutable
add_executable(mouth_node src/mouth_node.cpp)
ament_target_dependencies(mouth_node rclcpp std_msgs)

target_include_directories(mouth_node PRIVATE
  ${PORTAUDIO_INCLUDE_DIRS}
)

target_link_libraries(mouth_node
  ${PORTAUDIO_LIBRARIES}
  fftw3
)

# Instalación
install(TARGETS
  mouth_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
