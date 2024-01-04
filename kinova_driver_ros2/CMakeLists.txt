cmake_minimum_required(VERSION 3.8)
project(kinova_driver_ros2)

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
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(actionlib_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(kinova_msgs_ros2 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(angles REQUIRED)

find_package(Boost REQUIRED COMPONENTS thread)
if(Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIRS})
endif()

link_directories(${PROJECT_SOURCE_DIR}/lib/${CMAKE_LIBRARY_ARCHITECTURE})

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

# Add custom source here
add_executable(kinova_arm_driver
    src/nodes/kinova_arm_driver.cpp
    src/kinova_api.cpp
    src/kinova_arm.cpp
    src/kinova_comm.cpp
    src/kinova_tool_pose_action.cpp
    src/kinova_joint_angles_action.cpp
    src/kinova_fingers_action.cpp
    src/kinova_ros_types.cpp
    src/kinova_joint_trajectory_controller.cpp
    include/kinova_driver/kinova_joint_trajectory_controller.h
    include/kinova_driver/kinova_joint_angles_action.h
    include/kinova_driver/kinova_api.h
    include/kinova_driver/kinova_arm.h
    include/kinova_driver/kinova_comm.h
    include/kinova_driver/kinova_fingers_action.h
    include/kinova_driver/kinova_tool_pose_action.h
    include/kinova_driver/kinova_ros_types.h
    include/kinova/Kinova.API.USBCommLayerUbuntu.h
    include/kinova/Kinova.API.USBCommandLayerUbuntu.h
    include/kinova/KinovaTypes.h
)
# add the .h file form "./include" folder
target_include_directories(kinova_arm_driver
    PRIVATE include
)
# add source file form system
ament_target_dependencies( kinova_arm_driver 
    rclcpp
    rclcpp_action
    rclcpp_components
    actionlib_msgs
    std_msgs
    geometry_msgs
    trajectory_msgs
    kinova_msgs_ros2
    sensor_msgs
    control_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    interactive_markers
    visualization_msgs
    angles
    Boost)
# install this node to the system
install(TARGETS
    kinova_arm_driver
    DESTINATION lib/${PROJECT_NAME}
)


# Add custom source here
add_executable(kinova_tf_updater
    src/nodes/kinova_tf_updater.cpp
    src/kinova_ros_types.cpp
    src/kinova_arm_kinematics.cpp
    include/kinova_driver/kinova_tf_updater.h
    include/kinova_driver/kinova_ros_types.h
    include/kinova_driver/kinova_arm_kinematics.h
)
# add the .h file form "./include" folder
target_include_directories(kinova_tf_updater
    PRIVATE include
)
# add source file form system
ament_target_dependencies( kinova_tf_updater 
    rclcpp
    rclcpp_action
    rclcpp_components
    actionlib_msgs
    std_msgs
    geometry_msgs
    trajectory_msgs
    kinova_msgs_ros2
    sensor_msgs
    control_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    interactive_markers
    visualization_msgs
    angles
    Boost)
# install this node to the system
install(TARGETS
    kinova_tf_updater
    DESTINATION lib/${PROJECT_NAME}
)

# Add include folder to system
install(DIRECTORY include/
    DESTINATION share/${PROJECT_NAME}/
)

ament_package()





#############
## Install ##
#############

# Copy over Kinova .so binary blobs to /usr/lib
install(FILES
  lib/${CMAKE_LIBRARY_ARCHITECTURE}/USBCommLayerUbuntu.so
  lib/${CMAKE_LIBRARY_ARCHITECTURE}/USBCommandLayerUbuntu.so
  lib/${CMAKE_LIBRARY_ARCHITECTURE}/EthCommLayerUbuntu.so
  lib/${CMAKE_LIBRARY_ARCHITECTURE}/EthCommandLayerUbuntu.so
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.h"
#)


## Mark executables and/or libraries for installation
#install(TARGETS kinova_arm_driver kinova_tf_updater
#kinova_interactive_control kinova_driver
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

## Mark cpp header files for installation
#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.h"
#)
