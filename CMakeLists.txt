cmake_minimum_required(VERSION 3.8)
project(imu_ekf_ros)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ubica_rclcpp_utils REQUIRED)

#find_package (Eigen3 3.3 REQUIRED NO_MODULE)

set(ros_dependencies geometry_msgs nav_msgs rclcpp sensor_msgs std_msgs tf2 tf2_geometry_msgs tf2_ros ubica_rclcpp_utils)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/InitRequest.srv"
  DEPENDENCIES geometry_msgs std_msgs
)

include_directories(
  include
)

add_executable(imu_ekf_node src/ekf.cpp)
add_executable(init_imu_ekf_node src/init_ekf.cpp)
ament_target_dependencies(imu_ekf_node ${ros_dependencies})
ament_target_dependencies(init_imu_ekf_node ${ros_dependencies})
#target_link_libraries(imu_ekf_node Eigen3::Eigen)
#target_link_libraries(init_imu_ekf_node Eigen3::Eigen)
rosidl_target_interfaces(imu_ekf_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(init_imu_ekf_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS imu_ekf_node init_imu_ekf_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
