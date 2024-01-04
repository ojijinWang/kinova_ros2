cmake_minimum_required(VERSION 3.8)
project(kinova_msgs_ros2)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(actionlib_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)


#declare castom msg
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FingerPosition.msg"
  "msg/JointAngles.msg"
  "msg/JointVelocity.msg"
  "msg/JointTorque.msg"
  "msg/KinovaPose.msg"
  "msg/PoseVelocity.msg"
  "msg/PoseVelocityWithFingers.msg"
  "msg/PoseVelocityWithFingerVelocity.msg"
  "msg/CartesianForce.msg"
  "srv/Start.srv"
  "srv/Stop.srv"
  "srv/HomeArm.srv"
  "srv/SetForceControlParams.srv"
  "srv/SetEndEffectorOffset.srv"
  "srv/SetNullSpaceModeState.srv"
  "srv/SetTorqueControlMode.srv"
  "srv/SetTorqueControlParameters.srv"
  "srv/ClearTrajectories.srv"
  "srv/ZeroTorques.srv"
  "srv/AddPoseToCartesianTrajectory.srv"
  "srv/RunCOMParametersEstimation.srv"
  "action/ArmJointAngles.action"
  "action/ArmPose.action"
  "action/ArmKinovaPose.action"
  "action/SetFingersPosition.action"
  DEPENDENCIES std_msgs geometry_msgs actionlib_msgs
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



