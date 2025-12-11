cmake_minimum_required(VERSION 3.8)
project(imu_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ====================================
#     DEPENDENCIES
# ====================================

# ROS Core
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)


# ====================================
#     PYTHON NODES
# ====================================

install(PROGRAMS
  scripts/imu_raw_data_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)


# ========================================
#     LAUNCH
# ========================================

install(PARENT_DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
