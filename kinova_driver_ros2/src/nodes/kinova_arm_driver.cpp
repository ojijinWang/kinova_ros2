//============================================================================
// Name        : kinova_arm_driver.cpp
// Author      : WPI, Clearpath Robotics
// Version     : 0.5
// Copyright   : BSD
// Description : A ROS driver for controlling the Kinova Kinova robotic manipulator arm
//============================================================================

#include "kinova_driver/kinova_api.h"
#include "kinova_driver/kinova_arm.h"
#include "kinova_driver/kinova_tool_pose_action.h"
#include "kinova_driver/kinova_joint_angles_action.h"
#include "kinova_driver/kinova_fingers_action.h"
#include "kinova_driver/kinova_joint_trajectory_controller.h"


void auto_spin(const std::shared_ptr<rclcpp::Node>& node){
	rclcpp::spin(node);
}

int main(int argc, char **argv)
{
    // *** init node
	rclcpp::init(argc, argv);
	std::shared_ptr<rclcpp::Node> node_ = rclcpp::Node::make_shared("kinova_arm_driver");

    // *** activate printf() using launch
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    
    boost::recursive_mutex api_mutex;

    bool is_first_init = true;
    std::string kinova_robotType = "";
    std::string kinova_robotName = "";

    // Retrieve the (non-option) argument:
    node_->declare_parameter("robot_type", kinova_robotType);
    if (!node_->get_parameter("robot_type", kinova_robotType))
    {
        RCLCPP_ERROR(rclcpp::get_logger("kinova_arm_driver"), "No robot_type provided in the param!");
    }
    else // there is an input...
    {
        RCLCPP_INFO(rclcpp::get_logger("kinova_arm_driver"), "kinova_robotType is %s.", kinova_robotType.c_str());

        node_->declare_parameter("robot_name", kinova_robotName);
        if (!node_->get_parameter("robot_name", kinova_robotName))
        {
          kinova_robotName = kinova_robotType;
        }
        RCLCPP_INFO(rclcpp::get_logger("kinova_arm_driver"), "kinova_robotName is %s.", kinova_robotName.c_str());
    }

    // *** auto spin in a separate thread
	std::thread auto_spin_thread(auto_spin, node_);

	// *** the main loop
    while(rclcpp::ok())
    {
        try
        {
            kinova::KinovaComm comm(node_, api_mutex, is_first_init,kinova_robotType);
            kinova::KinovaArm kinova_arm(comm, node_, kinova_robotType, kinova_robotName);
            kinova::KinovaPoseActionServer pose_server(comm, node_, kinova_robotType, kinova_robotName);
            kinova::KinovaAnglesActionServer angles_server(comm, node_);
            kinova::KinovaFingersActionServer fingers_server(comm, node_);
            kinova::JointTrajectoryController joint_trajectory_controller(comm, node_);
            is_first_init = false;
            // *** loop for spin, actually useless because we use the auto spin using a new thread
            try{
                rclcpp::WallRate rate(100ms);
                while(rclcpp::ok())
                {
                    //rclcpp::spin_some(node_);
                    rate.sleep();
                }
            }
            catch(const std::exception& e){
                RCLCPP_INFO(rclcpp::get_logger("kinova_arm_driver"), "Error: '%s'", e.what());
            }
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("kinova_arm_driver"), e.what());
            kinova::KinovaAPI api;
            boost::recursive_mutex::scoped_lock lock(api_mutex);
            api.closeAPI();
            rclcpp::sleep_for(1s);
        }
    }
	

    rclcpp::shutdown();
    return 0;
}
