cmake_minimum_required(VERSION 3.8)
project(sensor_to_vins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(OpenCV REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(px4_ros_com REQUIRED)
find_package(px4_msgs REQUIRED)

add_executable(img_to_vins src/img_to_vins.cpp)
add_executable(imu_to_vins src/imu_to_vins.cpp)

ament_target_dependencies(
  img_to_vins 
  "rclcpp"
  "OpenCV"
  "sensor_msgs"
  "cv_bridge"
  )
ament_target_dependencies(
    imu_to_vins 
    "rclcpp"  
    "sensor_msgs"
    "px4_msgs"
    "px4_ros_com"
    )


install(TARGETS 
  img_to_vins imu_to_vins
  DESTINATION lib/${PROJECT_NAME})

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

ament_package()
