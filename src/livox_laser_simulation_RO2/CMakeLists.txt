cmake_minimum_required(VERSION 3.16)
project(ros2_livox_simulation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

###########
## Build ##
###########
add_executable(mid360_gz_bridge
  src/mid360_gz_bridge.cpp
)
target_include_directories(mid360_gz_bridge PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_compile_options(mid360_gz_bridge PRIVATE -Wall -Wextra -O3)
ament_target_dependencies(mid360_gz_bridge
  rclcpp
  ament_index_cpp
  sensor_msgs
  livox_ros_driver2
)
target_link_libraries(mid360_gz_bridge
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
)

#############
## Testing ##
#############
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_mid360_resampler
    test/test_mid360_resampler.cpp
  )
  if(TARGET test_mid360_resampler)
    target_include_directories(test_mid360_resampler PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    )
    target_compile_options(test_mid360_resampler PRIVATE -Wall -Wextra)
  endif()
endif()

#############
## Install ##
#############
install(TARGETS mid360_gz_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  scan_mode
  urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
