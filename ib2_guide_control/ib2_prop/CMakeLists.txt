cmake_minimum_required(VERSION 3.10.2)
project(ib2_prop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(WITH_PCA9685 "execute with PCA9685 board" OFF) # ON->OFF to OSS
if (WITH_PCA9685)
  add_definitions(-DWITH_PCA9685)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(ib2_msgs REQUIRED)

###########
## Build ##
###########

# Find libraries
include_directories(
 include
)

## Declare a C++ library
add_library(${PROJECT_NAME} SHARED
  src/prop_manager.cpp
  src/prop_tlmcmd.cpp
  src/prop_pca9685.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "ib2::PropManager"
  EXECUTABLE ib2_prop_node)

ament_target_dependencies(${PROJECT_NAME}
  "rclcpp"
  "rclcpp_components"
  "std_msgs"
  "example_interfaces"
  "ib2_msgs"
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY
  include/${PROJECT_NAME}
  DESTINATION include
)

install(DIRECTORY
  config DESTINATION share/${PROJECT_NAME}
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
