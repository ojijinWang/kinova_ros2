/*
 * kinova_tf_updater.h
 *
 *  Created on: Apr 16, 2013
 *      Author: mdedonato
 */

#ifndef KINOVA_DRIVER_KINOVA_TF_UPDATER_H
#define KINOVA_DRIVER_KINOVA_TF_UPDATER_H

#include <time.h>

#include <kinova_driver/kinova_arm_kinematics.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <kinova_msgs_ros2/msg/joint_angles.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;  // this is uesed in Subscriptor
using std::placeholders::_2;  // this is uesed in Service


namespace kinova
{

class KinovaTFTree
{
 public:
    explicit KinovaTFTree(const std::shared_ptr<rclcpp::Node> &node_handle, std::string& kinova_robotType);

 private:
    void jointAnglesMsgHandler(const kinova_msgs_ros2::msg::JointAngles::ConstSharedPtr& joint_angles);
    void calculatePostion(void);
    void tfUpdateHandler();

    kinova::KinovaKinematics kinematics_;
    kinova_msgs_ros2::msg::JointAngles current_angles_;
    rclcpp::Time last_angle_update_;
    rclcpp::Subscription<kinova_msgs_ros2::msg::JointAngles>::SharedPtr joint_angles_subscriber_;
    rclcpp::TimerBase::SharedPtr tf_update_timer_;
};

}  // namespace kinova
#endif  // KINOVA_DRIVER_KINOVA_TF_UPDATER_H
