cmake_minimum_required(VERSION 3.8)
project(ib2_platform_interfaces)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/CameraStatus.msg"
  "msg/ContainerStatus.msg"
  "msg/ManagerStatus.msg"
  "msg/MicrophoneStatus.msg"
  "msg/Mode.msg"
  "msg/MonitorStatus.msg"
  "msg/NodeStatusValue.msg"
  "msg/OperationType.msg"
  "msg/PowerStatus.msg"
  "msg/UserLogic.msg"
  "msg/UserNodeStatus.msg"
)
set(srv_files
  "srv/SetOperationType.srv"
  "srv/StopProcessingUserNode.srv"
  "srv/UserLogicCommand.srv"
  "srv/UserNodeCommand.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES builtin_interfaces
  ADD_LINTER_TESTS
)

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
