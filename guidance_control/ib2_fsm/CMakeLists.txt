cmake_minimum_required(VERSION 3.10.2)
project(ib2_fsm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ib2_ctl_common REQUIRED)
find_package(ib2_ctl REQUIRED)

find_package(Eigen3 REQUIRED)

###########
## Build ##
###########

# Find libraries
include_directories(
 include
 ${EIGEN3_INCLUDE_DIR}
)

# Library
add_library(${PROJECT_NAME} SHARED
  src/fsm.cpp
  src/thrust_allocator.cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(${PROJECT_NAME}_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}
  src/fsm.cpp
  src/thrust_allocator.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "ib2::Fsm"
  EXECUTABLE ib2_fsm_node)

ament_target_dependencies(${PROJECT_NAME}
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "example_interfaces"
  "geometry_msgs"
  "ib2_ctl_common"
  "ib2_ctl"
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

#############
## Install ##
#############

install(DIRECTORY
  include/${PROJECT_NAME}
  DESTINATION include
)

install(DIRECTORY
  config
  launch DESTINATION share/${PROJECT_NAME}
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
