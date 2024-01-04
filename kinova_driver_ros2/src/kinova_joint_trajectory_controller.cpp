#include "kinova_driver/kinova_joint_trajectory_controller.h"
#include <angles/angles.h>
#include <boost/lexical_cast.hpp>

using namespace kinova;

double toSec(builtin_interfaces::msg::Duration &time)
{
    double sec = (double)time.sec + (double)time.nanosec * 0.001;
    return sec;
}

JointTrajectoryController::JointTrajectoryController(kinova::KinovaComm &kinova_comm, std::shared_ptr<rclcpp::Node> &nh):
    kinova_comm_(kinova_comm),
    node_handle_(nh)
{
    //ROS_DEBUG_STREAM_ONCE("Get in: " << __PRETTY_FUNCTION__);

    std::string robot_name = "m1n6s300";
    std::string robot_type = "m1n6s300";
    if(!node_handle_->has_parameter("robot_name")){
        node_handle_->declare_parameter("robot_name", robot_name);
    }
    if(!node_handle_->has_parameter("robot_type")){
        node_handle_->declare_parameter("robot_type", robot_type);
    }
    node_handle_->get_parameter("robot_name", robot_name);
    node_handle_->get_parameter("robot_type", robot_type);
    number_joint_ = robot_type[3] - '0';
    prefix_ = robot_name;
    std::string name_ = robot_name + "_driver" + "/";

    sub_command_ = node_handle_->create_subscription<trajectory_msgs::msg::JointTrajectory>(name_ + "trajectory_controller/command", 1,
                                std::bind(&JointTrajectoryController::commandCB, this, _1));

    pub_joint_feedback_ = node_handle_->create_publisher<control_msgs::action::FollowJointTrajectory::Feedback>(name_ + "trajectory_controller/state", 1);
    pub_joint_velocity_ = node_handle_->create_publisher<kinova_msgs_ros2::msg::JointVelocity>(name_ + "in/joint_velocity", 1);

    traj_frame_id_ = "root";   
    joint_names_.resize(number_joint_);
    // std::cout << "joint names in feedback of trajectory state are: " << std::endl;
    for (uint i = 0; i<joint_names_.size(); i++)
    {
        joint_names_[i] = prefix_ + "_joint_" + boost::lexical_cast<std::string>(i+1);
        std::cout << joint_names_[i] << " ";
    }
    std::cout << std::endl;

    std::chrono::milliseconds msec = std::chrono::milliseconds { 10 };
    timer_pub_joint_vel_ = node_handle_->create_wall_timer(msec, std::bind(&JointTrajectoryController::pub_joint_vel, this));
    terminate_thread_ = false;

    thread_update_state_ = new boost::thread(boost::bind(&JointTrajectoryController::update_state, this));

    traj_feedback_msg_.joint_names.resize(joint_names_.size());
    traj_feedback_msg_.desired.positions.resize(joint_names_.size());
    traj_feedback_msg_.desired.velocities.resize(joint_names_.size());
    traj_feedback_msg_.actual.positions.resize(joint_names_.size());
    traj_feedback_msg_.actual.velocities.resize(joint_names_.size());
    traj_feedback_msg_.error.positions.resize(joint_names_.size());
    traj_feedback_msg_.error.velocities.resize(joint_names_.size());
    traj_feedback_msg_.joint_names = joint_names_;

    // counter in the timer to publish joint velocity command: pub_joint_vel()
    traj_command_points_index_ = 0;

    //ROS_DEBUG_STREAM_ONCE("Get out: " << __PRETTY_FUNCTION__);
}

JointTrajectoryController::~JointTrajectoryController()
{
    //ROS_DEBUG_STREAM_ONCE("Get in: " << __PRETTY_FUNCTION__);
    RCLCPP_WARN(rclcpp::get_logger("kinova_joint_trajectory_controller"), "destruction entered!");
    {
        boost::mutex::scoped_lock terminate_lock(terminate_thread_mutex_);
        terminate_thread_ = true;
    }

    //sub_command_.shutdown();
    //pub_joint_feedback_.shutdown();
    //pub_joint_velocity_.shutdown();

    timer_pub_joint_vel_->cancel();
    thread_update_state_->join();
    delete thread_update_state_;

    //ROS_DEBUG_STREAM_ONCE("Get out: " << __PRETTY_FUNCTION__);
}



void JointTrajectoryController::commandCB(const trajectory_msgs::msg::JointTrajectory::ConstSharedPtr traj_msg)
{
    //ROS_DEBUG_STREAM_ONCE("Get in: " << __PRETTY_FUNCTION__);

    bool command_abort = false;

//    // if receive new command, clear all trajectory and stop api
//    kinova_comm_.stopAPI();
//    if(!kinova_comm_.isStopped())
//    {
//        ros::Duration(0.01).sleep();
//    }
//    kinova_comm_.eraseAllTrajectories();

//    kinova_comm_.startAPI();
//    if(kinova_comm_.isStopped())
//    {
//        ros::Duration(0.01).sleep();
//    }

    traj_command_points_ = traj_msg->points;
    RCLCPP_INFO_STREAM(rclcpp::get_logger("kinova_joint_trajectory_controller"), "Trajectory controller Receive trajectory with points number: " << traj_command_points_.size());

    // Map the index in joint_names and the msg
    std::vector<int> lookup(number_joint_, -1);

    for (size_t j = 0; j<number_joint_; j++)
    {
        for (size_t k = 0; k<traj_msg->joint_names.size(); k++)
            if(traj_msg->joint_names[k] == joint_names_[j]) // find joint_j in msg;
            {
                lookup[j] = k;
                break;
            }

        if (lookup[j] == -1) // if joint_j not found in msg;
        {
            std::string error_msg = "Joint name : " + joint_names_[j] + " not found in the msg.";
            RCLCPP_ERROR(rclcpp::get_logger("kinova_joint_trajectory_controller"),"%s", error_msg.c_str());
            command_abort = true;
            return;
        }
    }

    // check msg validation
    for (size_t j = 0; j<traj_command_points_.size(); j++)
    {
        // position should not be empty
        if (traj_command_points_[j].positions.empty()) // find joint_j in msg;
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("kinova_joint_trajectory_controller"), "Positions in trajectory command cannot be empty at point: " << j);
            command_abort = true;
            break;
        }
        // position size match
        if (traj_command_points_[j].positions.size() != number_joint_)
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("kinova_joint_trajectory_controller"), "Positions at point " << j << " has size " << traj_command_points_[j].positions.size() << " in trajectory command, which does not match joint number! ");
            command_abort = true;
            break;
        }

        // if velocity provided, size match
        if (!traj_command_points_[j].velocities.empty() && traj_command_points_[j].velocities.size() != number_joint_)
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("kinova_joint_trajectory_controller"), "Velocities at point " << j << " has size " << traj_command_points_[j].velocities.size() << " in trajectory command, which does not match joint number! ");
            command_abort = true;
            break;
        }
    }

    if(command_abort)
        return;

    // store angle velocity command sent to robot
    // std::vector<KinovaAngles> kinova_angle_command;
    kinova_angle_command_.resize(traj_command_points_.size());
    for (size_t i = 0; i<traj_command_points_.size(); i++)
    {
        kinova_angle_command_[i].InitStruct(); // initial joint velocity to zeros.

        kinova_angle_command_[i].Actuator1 = traj_command_points_[i].velocities[0] *180/M_PI;
        kinova_angle_command_[i].Actuator2 = traj_command_points_[i].velocities[1] *180/M_PI;
        kinova_angle_command_[i].Actuator3 = traj_command_points_[i].velocities[2] *180/M_PI;
        kinova_angle_command_[i].Actuator4 = traj_command_points_[i].velocities[3] *180/M_PI;
        if (number_joint_>=6)
        {
            kinova_angle_command_[i].Actuator5 = traj_command_points_[i].velocities[4] *180/M_PI;
            kinova_angle_command_[i].Actuator6 = traj_command_points_[i].velocities[5] *180/M_PI;
            if (number_joint_==7)
                kinova_angle_command_[i].Actuator7 = traj_command_points_[i].velocities[6] *180/M_PI;
        }
    }

    std::vector<double> durations(traj_command_points_.size(), 0.0); // computed by time_from_start
    double trajectory_duration = toSec(traj_command_points_[0].time_from_start);

    durations[0] = trajectory_duration;
    // ROS_DEBUG_STREAM("durationsn 0 is: " << durations[0]);

    for (unsigned int i = 1; i<traj_command_points_.size(); i++)
    {
        durations[i] = toSec(traj_command_points_[i].time_from_start) - toSec(traj_command_points_[i-1].time_from_start);
        trajectory_duration += durations[i];
        // ROS_DEBUG_STREAM("durations " << i << " is: " << durations[i]);
    }

    // start timer thread to publish joint velocity command
    time_pub_joint_vel_ = rclcpp::Clock{}.now();
    timer_pub_joint_vel_->reset();

    //ROS_DEBUG_STREAM_ONCE("Get out: " << __PRETTY_FUNCTION__);
}


void JointTrajectoryController::pub_joint_vel()
{
    // send out each velocity command with corresponding duration delay.

    kinova_msgs_ros2::msg::JointVelocity joint_velocity_msg;

    if (traj_command_points_index_ <  (int)kinova_angle_command_.size() && rclcpp::ok())
    {
        joint_velocity_msg.joint1 = kinova_angle_command_[traj_command_points_index_].Actuator1;
        joint_velocity_msg.joint2 = kinova_angle_command_[traj_command_points_index_].Actuator2;
        joint_velocity_msg.joint3 = kinova_angle_command_[traj_command_points_index_].Actuator3;
        joint_velocity_msg.joint4 = kinova_angle_command_[traj_command_points_index_].Actuator4;
        joint_velocity_msg.joint5 = kinova_angle_command_[traj_command_points_index_].Actuator5;
        joint_velocity_msg.joint6 = kinova_angle_command_[traj_command_points_index_].Actuator6;
        joint_velocity_msg.joint7 = kinova_angle_command_[traj_command_points_index_].Actuator7;

        // In debug: compare values with topic: follow_joint_trajectory/goal, command
//        ROS_DEBUG_STREAM_ONCE( std::endl <<" joint_velocity_msg.joint1: " << joint_velocity_msg.joint1 * M_PI/180 <<
//                          std::endl <<" joint_velocity_msg.joint2: " << joint_velocity_msg.joint2 * M_PI/180 <<
//                          std::endl <<" joint_velocity_msg.joint3: " << joint_velocity_msg.joint3 * M_PI/180 <<
//                          std::endl <<" joint_velocity_msg.joint4: " << joint_velocity_msg.joint4 * M_PI/180 <<
//                          std::endl <<" joint_velocity_msg.joint5: " << joint_velocity_msg.joint5 * M_PI/180 <<
//                          std::endl <<" joint_velocity_msg.joint6: " << joint_velocity_msg.joint6 * M_PI/180 );

        pub_joint_velocity_->publish(joint_velocity_msg);

        if( (rclcpp::Clock{}.now() - time_pub_joint_vel_) >= traj_command_points_[traj_command_points_index_].time_from_start)
        {
            RCLCPP_INFO_STREAM(rclcpp::get_logger("kinova_joint_trajectory_controller"), "Moved to point " << traj_command_points_index_++);
        }
    }
    else // if come accross all the points, then stop timer.
    {
        joint_velocity_msg.joint1 = 0;
        joint_velocity_msg.joint2 = 0;
        joint_velocity_msg.joint3 = 0;
        joint_velocity_msg.joint4 = 0;
        joint_velocity_msg.joint5 = 0;
        joint_velocity_msg.joint6 = 0;
        joint_velocity_msg.joint7 = 0;

        traj_command_points_.clear();

        traj_command_points_index_ = 0;
        timer_pub_joint_vel_->cancel();
    }
}

void JointTrajectoryController::update_state()
{
    //ROS_DEBUG_STREAM_ONCE("Get in: " << __PRETTY_FUNCTION__);

    rclcpp::WallRate update_rate(10);
    previous_pub_ = rclcpp::Clock{}.now();
    while (rclcpp::ok())
    {
        // check if terminate command is sent from main thread
        {
            boost::mutex::scoped_lock terminate_lock(terminate_thread_mutex_);
            if (terminate_thread_)
            {
                break;
            }
        }

        traj_feedback_msg_.header.frame_id = traj_frame_id_;
        traj_feedback_msg_.header.stamp = rclcpp::Clock{}.now();
        KinovaAngles current_joint_angles;
        KinovaAngles current_joint_velocity;
        AngularPosition current_joint_command;

        kinova_comm_.getAngularCommand(current_joint_command);
        kinova_comm_.getJointAngles(current_joint_angles);
        kinova_comm_.getJointVelocities(current_joint_velocity);


        traj_feedback_msg_.desired.positions[0] = current_joint_command.Actuators.Actuator1 *M_PI/180;
        traj_feedback_msg_.desired.positions[1] = current_joint_command.Actuators.Actuator2 *M_PI/180;
        traj_feedback_msg_.desired.positions[2] = current_joint_command.Actuators.Actuator3 *M_PI/180;
        traj_feedback_msg_.desired.positions[3] = current_joint_command.Actuators.Actuator4 *M_PI/180;
        if (number_joint_>=6)
        {
            traj_feedback_msg_.desired.positions[4] = current_joint_command.Actuators.Actuator5 *M_PI/180;
            traj_feedback_msg_.desired.positions[5] = current_joint_command.Actuators.Actuator6 *M_PI/180;
            if (number_joint_==7)
                traj_feedback_msg_.desired.positions[6] = current_joint_command.Actuators.Actuator7 *M_PI/180;
        }

        traj_feedback_msg_.actual.positions[0] = current_joint_angles.Actuator1 *M_PI/180;
        traj_feedback_msg_.actual.positions[1] = current_joint_angles.Actuator2 *M_PI/180;
        traj_feedback_msg_.actual.positions[2] = current_joint_angles.Actuator3 *M_PI/180;
        traj_feedback_msg_.actual.positions[3] = current_joint_angles.Actuator4 *M_PI/180;
        if (number_joint_>=6)
        {
            traj_feedback_msg_.actual.positions[4] = current_joint_angles.Actuator5 *M_PI/180;
            traj_feedback_msg_.actual.positions[5] = current_joint_angles.Actuator6 *M_PI/180;
            if (number_joint_==7)
                traj_feedback_msg_.actual.positions[6] = current_joint_angles.Actuator7 *M_PI/180;
        }

        traj_feedback_msg_.actual.velocities[0] = current_joint_velocity.Actuator1 *M_PI/180;
        traj_feedback_msg_.actual.velocities[1] = current_joint_velocity.Actuator2 *M_PI/180;
        traj_feedback_msg_.actual.velocities[2] = current_joint_velocity.Actuator3 *M_PI/180;
        traj_feedback_msg_.actual.velocities[3] = current_joint_velocity.Actuator4 *M_PI/180;
        if (number_joint_>=6)
        {
            traj_feedback_msg_.actual.velocities[4] = current_joint_velocity.Actuator5 *M_PI/180;
            traj_feedback_msg_.actual.velocities[5] = current_joint_velocity.Actuator6 *M_PI/180;
            if (number_joint_==7)
                traj_feedback_msg_.actual.velocities[6] = current_joint_velocity.Actuator7 *M_PI/180;
        }

        for (size_t j = 0; j<joint_names_.size(); j++)
        {
            traj_feedback_msg_.error.positions[j] = traj_feedback_msg_.actual.positions[j] - traj_feedback_msg_.desired.positions[j];
        }

        //        ROS_WARN_STREAM("I'm publishing after second: " << (ros::Time::now() - previous_pub_).toSec());
        pub_joint_feedback_->publish(traj_feedback_msg_);
        previous_pub_ = rclcpp::Clock{}.now();
        update_rate.sleep();
    }
    //ROS_DEBUG_STREAM_ONCE("Get out: " << __PRETTY_FUNCTION__);
}
