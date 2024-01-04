#ifndef KINOVA_JOINT_TRAJECTORY_CONTROLLER_H
#define KINOVA_JOINT_TRAJECTORY_CONTROLLER_H


#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <builtin_interfaces/msg/duration.hpp>
#include <kinova_msgs_ros2/msg/joint_velocity.hpp>
#include <boost/thread.hpp>

using std::placeholders::_1;  // this is uesed in Subscriptor
using std::placeholders::_2;  // this is uesed in Service

#include "kinova_ros_types.h"
#include "kinova_comm.h"
#include "kinova_api.h"

namespace kinova
{

class JointTrajectoryController
{
public:
    JointTrajectoryController(kinova::KinovaComm &kinova_comm, std::shared_ptr<rclcpp::Node> &nh);
    ~JointTrajectoryController();


private:
    std::shared_ptr<rclcpp::Node> node_handle_;

    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr sub_command_;
    rclcpp::Publisher<control_msgs::action::FollowJointTrajectory::Feedback>::SharedPtr pub_joint_feedback_;
    rclcpp::Publisher<kinova_msgs_ros2::msg::JointVelocity>::SharedPtr pub_joint_velocity_;

    rclcpp::Time previous_pub_;
    rclcpp::Time time_pub_joint_vel_;

    rclcpp::TimerBase::SharedPtr timer_pub_joint_vel_;
    boost::mutex terminate_thread_mutex_;
    boost::thread* thread_update_state_;
    bool terminate_thread_;

    builtin_interfaces::msg::Duration time_from_start_;
    sensor_msgs::msg::JointState current_joint_state_;

    KinovaComm kinova_comm_;
    TrajectoryPoint kinova_traj_point_;

    //trajectory_msgs::JointTrajectory joint_traj_;
    //trajectory_msgs::JointTrajectoryPoint joint_traj_point_;
    std::string traj_frame_id_;
    std::vector<trajectory_msgs::msg::JointTrajectoryPoint> traj_command_points_;
    control_msgs::action::FollowJointTrajectory::Feedback traj_feedback_msg_;

    // stores the command to send to robot, in Kinova type (KinovaAngles)
    std::vector<KinovaAngles> kinova_angle_command_;

    uint number_joint_;
    int traj_command_points_index_;
    std::vector<std::string> joint_names_;
    std::string prefix_;

    struct Segment
    {
        double start_time;
        double duration;
        std::vector<double> positions;
        std::vector<double> velocities;
    };


    // call back function when receive a trajectory command
    void commandCB(const trajectory_msgs::msg::JointTrajectory::ConstSharedPtr traj_msg);

    // reflash the robot state and publish the joint state: either by timer or thread
    void update_state(); // by thread

    void pub_joint_vel(); // by timer
    int test;

};






}


#endif // KINOVA_JOINT_TRAJECTORY_CONTROLLER_H
