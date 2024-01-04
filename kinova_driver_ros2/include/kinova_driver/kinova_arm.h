/*
 * kinova_arm_driver.h
 *
 *  Created on: Feb 26, 2013
 *  Modified on: June 25, 2013
 *      Author: mdedonato, Clearpath Robotics
 *
 */

#ifndef KINOVA_DRIVER_KINOVA_ARM_H
#define KINOVA_DRIVER_KINOVA_ARM_H

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <kinova_msgs_ros2/srv/stop.hpp>
#include <kinova_msgs_ros2/srv/start.hpp>
#include <kinova_msgs_ros2/srv/home_arm.hpp>
#include <kinova_msgs_ros2/msg/joint_velocity.hpp>
#include <kinova_msgs_ros2/msg/pose_velocity.hpp>
#include <kinova_msgs_ros2/msg/pose_velocity_with_fingers.hpp>
#include <kinova_msgs_ros2/msg/pose_velocity_with_finger_velocity.hpp>
#include <kinova_msgs_ros2/msg/joint_torque.hpp>
#include <kinova_msgs_ros2/msg/finger_position.hpp>
#include <kinova_msgs_ros2/msg/joint_angles.hpp>
#include <kinova_msgs_ros2/msg/kinova_pose.hpp>
#include <kinova_msgs_ros2/srv/set_force_control_params.hpp>
#include <kinova_msgs_ros2/srv/set_end_effector_offset.hpp>
#include <kinova_msgs_ros2/srv/set_null_space_mode_state.hpp>
#include <kinova_msgs_ros2/srv/set_torque_control_mode.hpp>
#include <kinova_msgs_ros2/srv/set_torque_control_parameters.hpp>
#include <kinova_msgs_ros2/srv/clear_trajectories.hpp>
#include <kinova_msgs_ros2/srv/add_pose_to_cartesian_trajectory.hpp>
#include <kinova_msgs_ros2/srv/zero_torques.hpp>
#include <kinova_msgs_ros2/srv/run_com_parameters_estimation.hpp>
#include <kinova_msgs_ros2/msg/cartesian_force.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;  // this is uesed in Subscriptor
using std::placeholders::_2;  // this is uesed in Service

#include <time.h>
#include <math.h>
#include <vector>

#include "kinova/KinovaTypes.h"
#include "kinova_driver/kinova_comm.h"
#include "kinova_driver/kinova_api.h"


namespace kinova
{

struct robot_info
{
    int id;
    std::string name;
    std::string type;
    std::string serial;
};

class KinovaArm
{
 public:
    KinovaArm(KinovaComm& arm, const std::shared_ptr<rclcpp::Node> &node_handle, const std::string &kinova_robotType, const std::string &kinova_robotName);
    ~KinovaArm();

    //Subscriber callbacks --------------------------------------------------------
    void jointVelocityCallback(const kinova_msgs_ros2::msg::JointVelocity::ConstSharedPtr joint_vel);
    void cartesianVelocityCallback(const kinova_msgs_ros2::msg::PoseVelocity::ConstSharedPtr cartesian_vel);
    void cartesianVelocityWithFingersCallback(const kinova_msgs_ros2::msg::PoseVelocityWithFingers::ConstSharedPtr cartesian_vel_with_fingers);
    void cartesianVelocityWithFingerVelocityCallback(const kinova_msgs_ros2::msg::PoseVelocityWithFingerVelocity::ConstSharedPtr cartesian_vel_with_finger_velocity);
    void jointTorqueSubscriberCallback(const kinova_msgs_ros2::msg::JointTorque::ConstSharedPtr joint_torque);
    void forceSubscriberCallback(const kinova_msgs_ros2::msg::CartesianForce::ConstSharedPtr force);

    // Service callbacks -----------------------------------------------------------
    bool stopServiceCallback(std::shared_ptr<kinova_msgs_ros2::srv::Stop::Request> req, std::shared_ptr<kinova_msgs_ros2::srv::Stop::Response> res);
    bool startServiceCallback(std::shared_ptr<kinova_msgs_ros2::srv::Start::Request> req, std::shared_ptr<kinova_msgs_ros2::srv::Start::Response> res);
    bool homeArmServiceCallback(std::shared_ptr<kinova_msgs_ros2::srv::HomeArm::Request> req, std::shared_ptr<kinova_msgs_ros2::srv::HomeArm::Response> res);
    bool ActivateNullSpaceModeCallback(std::shared_ptr<kinova_msgs_ros2::srv::SetNullSpaceModeState::Request> req,
                                       std::shared_ptr<kinova_msgs_ros2::srv::SetNullSpaceModeState::Response> res);
    bool addCartesianPoseToTrajectory(std::shared_ptr<kinova_msgs_ros2::srv::AddPoseToCartesianTrajectory::Request> req,
                                std::shared_ptr<kinova_msgs_ros2::srv::AddPoseToCartesianTrajectory::Response> res);
    bool clearTrajectoriesServiceCallback(std::shared_ptr<kinova_msgs_ros2::srv::ClearTrajectories::Request> req,
                                          std::shared_ptr<kinova_msgs_ros2::srv::ClearTrajectories::Response> res);
    bool setEndEffectorOffsetCallback(std::shared_ptr<kinova_msgs_ros2::srv::SetEndEffectorOffset::Request> req,
                                      std::shared_ptr<kinova_msgs_ros2::srv::SetEndEffectorOffset::Response> res);

    //Torque control
    bool setForceControlParamsCallback(std::shared_ptr<kinova_msgs_ros2::srv::SetForceControlParams::Request> req,
                                       std::shared_ptr<kinova_msgs_ros2::srv::SetForceControlParams::Response> res);
    bool startForceControlCallback(std::shared_ptr<kinova_msgs_ros2::srv::Start::Request> req,
                                   std::shared_ptr<kinova_msgs_ros2::srv::Start::Response> res);
    bool stopForceControlCallback(std::shared_ptr<kinova_msgs_ros2::srv::Stop::Request> req,
                                  std::shared_ptr<kinova_msgs_ros2::srv::Stop::Response> res);
    bool setTorqueControlModeService(std::shared_ptr<kinova_msgs_ros2::srv::SetTorqueControlMode::Request> req,
                                     std::shared_ptr<kinova_msgs_ros2::srv::SetTorqueControlMode::Response> res);
    bool setTorqueControlParametersService(std::shared_ptr<kinova_msgs_ros2::srv::SetTorqueControlParameters::Request> req,
                                           std::shared_ptr<kinova_msgs_ros2::srv::SetTorqueControlParameters::Response> res);
    bool setJointTorquesToZeroService(std::shared_ptr<kinova_msgs_ros2::srv::ZeroTorques::Request> req,
                                      std::shared_ptr<kinova_msgs_ros2::srv::ZeroTorques::Response> res);
    bool runCOMParameterEstimationService(std::shared_ptr<kinova_msgs_ros2::srv::RunCOMParametersEstimation::Request> req,
                                          std::shared_ptr<kinova_msgs_ros2::srv::RunCOMParametersEstimation::Response> res);

 private:
    void positionTimer(); //void positionTimer(const ros::TimerEvent& e);
    void cartesianVelocityTimer();
    void jointVelocityTimer();
    void statusTimer();

    void publishJointAngles(void);
    void publishToolPosition(void);
    void publishToolWrench(void);
    void publishFingerPosition(void);

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<rclcpp::Node> node_handle_;
    KinovaComm &kinova_comm_;

    // Publishers, subscribers, services
    rclcpp::Subscription<kinova_msgs_ros2::msg::JointVelocity>::SharedPtr joint_velocity_subscriber_;
    rclcpp::Subscription<kinova_msgs_ros2::msg::PoseVelocity>::SharedPtr cartesian_velocity_subscriber_;
    rclcpp::Subscription<kinova_msgs_ros2::msg::PoseVelocityWithFingers>::SharedPtr cartesian_velocity_with_fingers_subscriber_;
    rclcpp::Subscription<kinova_msgs_ros2::msg::PoseVelocityWithFingerVelocity>::SharedPtr cartesian_velocity_with_finger_velocity_subscriber_;
    rclcpp::Subscription<kinova_msgs_ros2::msg::JointTorque>::SharedPtr joint_torque_subscriber_;
    rclcpp::Subscription<kinova_msgs_ros2::msg::CartesianForce>::SharedPtr cartesian_force_subscriber_;

    rclcpp::Publisher<kinova_msgs_ros2::msg::JointAngles>::SharedPtr joint_angles_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr tool_position_publisher_;
    rclcpp::Publisher<kinova_msgs_ros2::msg::JointAngles>::SharedPtr joint_torque_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr tool_wrench_publisher_;
    rclcpp::Publisher<kinova_msgs_ros2::msg::FingerPosition>::SharedPtr finger_position_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
    rclcpp::Publisher<kinova_msgs_ros2::msg::JointAngles>::SharedPtr joint_command_publisher_;
    rclcpp::Publisher<kinova_msgs_ros2::msg::KinovaPose>::SharedPtr cartesian_command_publisher_;

    rclcpp::Service<kinova_msgs_ros2::srv::Stop>::SharedPtr stop_service_;
    rclcpp::Service<kinova_msgs_ros2::srv::Start>::SharedPtr start_service_;
    rclcpp::Service<kinova_msgs_ros2::srv::HomeArm>::SharedPtr homing_service_;
    rclcpp::Service<kinova_msgs_ros2::srv::SetNullSpaceModeState>::SharedPtr start_null_space_service_;
    rclcpp::Service<kinova_msgs_ros2::srv::AddPoseToCartesianTrajectory>::SharedPtr add_trajectory_;
    rclcpp::Service<kinova_msgs_ros2::srv::ClearTrajectories>::SharedPtr clear_trajectories_;

    rclcpp::Service<kinova_msgs_ros2::srv::SetTorqueControlMode>::SharedPtr set_torque_control_mode_service_;
    rclcpp::Service<kinova_msgs_ros2::srv::SetTorqueControlParameters>::SharedPtr set_torque_control_parameters_service_;
    rclcpp::Service<kinova_msgs_ros2::srv::ZeroTorques>::SharedPtr set_actuator_torques_to_zero_;
    rclcpp::Service<kinova_msgs_ros2::srv::SetForceControlParams>::SharedPtr set_force_control_params_service_;
    rclcpp::Service<kinova_msgs_ros2::srv::Start>::SharedPtr start_force_control_service_;
    rclcpp::Service<kinova_msgs_ros2::srv::Stop>::SharedPtr stop_force_control_service_;
    rclcpp::Service<kinova_msgs_ros2::srv::RunCOMParametersEstimation>::SharedPtr run_COM_parameter_estimation_service_;

    rclcpp::Service<kinova_msgs_ros2::srv::SetEndEffectorOffset>::SharedPtr set_end_effector_offset_service_;

    // Timers for control loops
    rclcpp::TimerBase::SharedPtr status_timer_;

    // Parameters
    std::string kinova_robotType_;
    std::string kinova_robotName_;
    std::string tf_prefix_;

    char robot_category_;
    int robot_category_version_;
    char wrist_type_;
    int arm_joint_number_;
    char robot_mode_;
    int finger_number_;
    int joint_total_number_;
    ROBOT_TYPE robot_type_;


    double status_interval_seconds_;
    double finger_conv_ratio_;
    bool convert_joint_velocities_;

    // State tracking or utility members
    AngularInfo joint_velocities_;
    float l_joint_torque_[COMMAND_SIZE];
    float l_force_cmd_[COMMAND_SIZE];
    CartesianInfo cartesian_velocities_;

    std::vector< std::string > joint_names_;

    //multiple robots
    int active_robot_id_;
    std::vector<robot_info> robots_;

};


}  // namespace kinova
#endif  // KINOVA_DRIVER_KINOVA_ARM_H
