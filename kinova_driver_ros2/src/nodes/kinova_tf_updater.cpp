/*
 * kinova_tf_updater.cpp

 *
 *  Created on: Apr 16, 2013
 *      Author: mdedonato
 */

#include "kinova_driver/kinova_tf_updater.h"
#include "kinova_driver/kinova_ros_types.h"

namespace kinova
{

KinovaTFTree::KinovaTFTree(const std::shared_ptr<rclcpp::Node> &node_handle, std::string& kinova_robotType)
    : kinematics_(node_handle, kinova_robotType)
{
    std::string name_ = kinova_robotType + "_driver" + "/";
    joint_angles_subscriber_ = node_handle->create_subscription<kinova_msgs_ros2::msg::JointAngles>(name_ + "in/joint_angles", 1,
                                std::bind(&KinovaTFTree::jointAnglesMsgHandler, this, _1));

    current_angles_.joint1 = 0;
    current_angles_.joint2 = 0;
    current_angles_.joint3 = 0;
    current_angles_.joint4 = 0;
    current_angles_.joint5 = 0;
    current_angles_.joint6 = 0;
    current_angles_.joint7 = 0;
    last_angle_update_ = rclcpp::Clock{}.now();

    std::chrono::milliseconds msec = std::chrono::milliseconds { 10 };
    tf_update_timer_ = node_handle->create_wall_timer(msec, std::bind(&KinovaTFTree::tfUpdateHandler, this));
    tf_update_timer_->cancel();
}


void KinovaTFTree::jointAnglesMsgHandler(const kinova_msgs_ros2::msg::JointAngles::ConstSharedPtr& joint_angles)
{
    current_angles_.joint1 = joint_angles->joint1;
    current_angles_.joint2 = joint_angles->joint2;
    current_angles_.joint3 = joint_angles->joint3;
    current_angles_.joint4 = joint_angles->joint4;
    current_angles_.joint5 = joint_angles->joint5;
    current_angles_.joint6 = joint_angles->joint6;
    current_angles_.joint7 = joint_angles->joint7;
    last_angle_update_ = rclcpp::Clock{}.now();
    tf_update_timer_->reset();
}


void KinovaTFTree::calculatePostion(void)
{
    // Update the forward Kinematics
    float Q[7] = {kinematics_.degToRad(current_angles_.joint1),
                 kinematics_.degToRad(current_angles_.joint2),
                 kinematics_.degToRad(current_angles_.joint3),
                 kinematics_.degToRad(current_angles_.joint4),
                 kinematics_.degToRad(current_angles_.joint5),
                 kinematics_.degToRad(current_angles_.joint6),
                 kinematics_.degToRad(current_angles_.joint7)};

    kinematics_.updateForward(Q);
}


void KinovaTFTree::tfUpdateHandler()
{
    this->calculatePostion();  // Update TF Tree

    if ((rclcpp::Clock{}.now().seconds() - last_angle_update_.seconds()) > 1)
    {
        tf_update_timer_->cancel();
    }
}

}  // namespace kinova

void auto_spin(const std::shared_ptr<rclcpp::Node>& node){
	rclcpp::spin(node);
}
int main(int argc, char **argv)
{
    /* Set up ROS */
    rclcpp::init(argc, argv);
	std::shared_ptr<rclcpp::Node> node_ = rclcpp::Node::make_shared("kinova_tf_updater");

    std::string kinova_robotType = "";

    // Retrieve the (non-option) argument:
    node_->declare_parameter("robot_type", kinova_robotType);
    if (!node_->get_parameter("robot_type", kinova_robotType))
    {
        RCLCPP_ERROR(rclcpp::get_logger("kinova_tf_updater"), "No robot_type provided in the param!");
    }
    else // there is an input...
    {
        RCLCPP_INFO(rclcpp::get_logger("kinova_tf_updater"), "kinova_robotType is %s.", kinova_robotType.c_str());
    }

    // *** auto spin in a separate thread
	std::thread auto_spin_thread(auto_spin, node_);

    kinova::KinovaTFTree KinovaTF(node_, kinova_robotType);
    // *** the main loop
	try{
		rclcpp::WallRate rate(10ms);
		while(rclcpp::ok())
		{
           rate.sleep();
        }
    }
    catch(const std::exception& e){
		RCLCPP_INFO(rclcpp::get_logger("kinova_tf_updater"), "Error: '%s'", e.what());
	}

    rclcpp::shutdown();
    return 0;
}
