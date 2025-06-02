cmake_minimum_required(VERSION 3.10.2)
project(ib2_ctl_common)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

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
  src/Constants.cpp
  src/FileReader.cpp
  src/FileWriter.cpp
  src/IntOverflow.cpp
  src/Interpolator.cpp
  src/Log.cpp
  src/Mjd.cpp
  src/MovingAverage.cpp
  src/RangeChecker.cpp
  src/Solver1.cpp
  src/StopWatch.cpp
  src/TaiUtc.cpp
  src/Utility.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  "rclcpp"
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
