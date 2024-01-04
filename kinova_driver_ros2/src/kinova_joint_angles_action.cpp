/**
 *      _____
 *     /  _  \
 *    / _/ \  \
 *   / / \_/   \
 *  /  \_/  _   \  ___  _    ___   ___   ____   ____   ___   _____  _   _
 *  \  / \_/ \  / /  _\| |  | __| / _ \ | ++ \ | ++ \ / _ \ |_   _|| | | |
 *   \ \_/ \_/ /  | |  | |  | ++ | |_| || ++ / | ++_/| |_| |  | |  | +-+ |
 *    \  \_/  /   | |_ | |_ | ++ |  _  || |\ \ | |   |  _  |  | |  | +-+ |
 *     \_____/    \___/|___||___||_| |_||_| \_\|_|   |_| |_|  |_|  |_| |_|
 *             ROBOTICS™
 *
 *  File: kinova_joint_angles_action.cpp
 *  Desc: Class for moving/querying kinova arm.
 *  Auth: Alex Bencz, Jeff Schmidt
 *
 *  Copyright (c) 2013, Clearpath Robotics, Inc.
 *  All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Clearpath Robotics, Inc. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Please send comments, questions, or patches to skynet@clearpathrobotics.com
 *
 */


#include "kinova_driver/kinova_joint_angles_action.h"
#include "kinova_driver/kinova_ros_types.h"


namespace kinova
{

KinovaAnglesActionServer::KinovaAnglesActionServer(KinovaComm &arm_comm, const std::shared_ptr<rclcpp::Node> &nh)
    : arm_comm_(arm_comm),
      node_handle_(nh)
{
    stall_interval_seconds_ = 0.5;
    stall_threshold_ = 1.0;
    rate_hz_ = 10.0;
    double tolerance = 2.0;
    jointSpeedLimitJoints123 = 20;
    jointSpeedLimitJoints456 = 20;
    if(!node_handle_->has_parameter("stall_interval_seconds")){
        node_handle_->declare_parameter("stall_interval_seconds", stall_interval_seconds_);
    }
    if(!node_handle_->has_parameter("stall_threshold")){
        node_handle_->declare_parameter("stall_threshold", stall_threshold_);
    }
    if(!node_handle_->has_parameter("rate_hz")){
        node_handle_->declare_parameter("rate_hz", rate_hz_);
    }
    if(!node_handle_->has_parameter("tolerance")){
        node_handle_->declare_parameter("tolerance", tolerance);
    }
    if(!node_handle_->has_parameter("jointSpeedLimitParameter1")){
        node_handle_->declare_parameter("jointSpeedLimitParameter1", jointSpeedLimitJoints123);
    }
    if(!node_handle_->has_parameter("jointSpeedLimitParameter2")){
        node_handle_->declare_parameter("jointSpeedLimitParameter2", jointSpeedLimitJoints456);
    }
    node_handle_->get_parameter("stall_interval_seconds", stall_interval_seconds_);
    node_handle_->get_parameter("stall_threshold", stall_threshold_);
    node_handle_->get_parameter("rate_hz", rate_hz_);
    node_handle_->get_parameter("tolerance", tolerance);
    node_handle_->get_parameter("jointSpeedLimitParameter1", jointSpeedLimitJoints123);
    node_handle_->get_parameter("jointSpeedLimitParameter2", jointSpeedLimitJoints456);
    tolerance_ = (float)tolerance;

    std::string robot_name = "m1n6s300";
    if(!node_handle_->has_parameter("robot_name")){
        node_handle_->declare_parameter("robot_name", robot_name);
    }
    node_handle_->get_parameter("robot_name", robot_name);

    std::string name_ = robot_name + "_driver" + "/";
    this->action_server_ = rclcpp_action::create_server<kinova_msgs_ros2::action::ArmJointAngles>(node_handle_, name_ + "joint_angles",
                        std::bind(&KinovaAnglesActionServer::handle_goal, this, _1, _2),
                        std::bind(&KinovaAnglesActionServer::handle_cancel, this, _1),
                        std::bind(&KinovaAnglesActionServer::handle_accepted, this, _1));
}


KinovaAnglesActionServer::~KinovaAnglesActionServer()
{
    RCLCPP_WARN(rclcpp::get_logger("kinova_joint_angles_action"), "destruction entered!");
}


void KinovaAnglesActionServer::execute(const std::shared_ptr<GoalHandle_Kinova_Action> goal_handle)
{
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Kinova_Action::Feedback>();
    auto result = std::make_shared<Kinova_Action::Result>();
    KinovaAngles current_joint_angles;
    rclcpp::Time current_time = rclcpp::Clock{}.now();

    bool action_is_over = false;

    try
    {
        arm_comm_.getJointAngles(current_joint_angles);

        if (arm_comm_.isStopped())
        {
            RCLCPP_INFO(rclcpp::get_logger("kinova_joint_angles_action"), "Could not complete joint angle action because the arm is 'stopped'.");
            result->angles = current_joint_angles.constructAnglesMsg();
            goal_handle->abort(result);
            RCLCPP_WARN_STREAM(rclcpp::get_logger("kinova_joint_angles_action"), __PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ", setAborted ");
            return;
        }

        last_nonstall_time_ = current_time;
        last_nonstall_angles_ = current_joint_angles;

        KinovaAngles target(goal->angles);
        arm_comm_.setJointAngles(target,jointSpeedLimitJoints123,jointSpeedLimitJoints456);

        // Loop until the action completed, is preempted, or fails in some way.
        // timeout is left to the caller since the timeout may greatly depend on
        // the context of the movement.
        while (!action_is_over)
        {
            rclcpp::spin_some(node_handle_);
	        if (arm_comm_.isStopped())
            {
                result->angles = current_joint_angles.constructAnglesMsg();
                goal_handle->abort(result);
                RCLCPP_WARN_STREAM(rclcpp::get_logger("kinova_joint_angles_action"), __PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ", setAborted ");
                action_is_over = true;
            }
            else if (goal_handle->is_canceling() || !rclcpp::ok())
            {
                result->angles = current_joint_angles.constructAnglesMsg();
                arm_comm_.stopAPI();
                arm_comm_.startAPI();
                goal_handle->canceled(result);
                RCLCPP_WARN_STREAM(rclcpp::get_logger("kinova_joint_angles_action"), __PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ", setPreempted ");
                action_is_over = true;
            }

            arm_comm_.getJointAngles(current_joint_angles);
            current_time = rclcpp::Clock{}.now();
            feedback->angles = current_joint_angles.constructAnglesMsg();
//            action_server_.publishFeedback(feedback);

            if (target.isCloseToOther(current_joint_angles, tolerance_))
            {
                // Check if the action has succeeeded
                result->angles = current_joint_angles.constructAnglesMsg();
                goal_handle->succeed(result);
                RCLCPP_WARN_STREAM(rclcpp::get_logger("kinova_joint_angles_action"), __PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ", setSucceeded ");
                action_is_over = true;
            }
            else if (!last_nonstall_angles_.isCloseToOther(current_joint_angles, stall_threshold_))
            {
                // Check if we are outside of a potential stall condition
                last_nonstall_time_ = current_time;
                last_nonstall_angles_ = current_joint_angles;
            }
            else if ((current_time - last_nonstall_time_).seconds() > stall_interval_seconds_)
            {
                // Check if the full stall condition has been meet
                result->angles = current_joint_angles.constructAnglesMsg();
                if (!arm_comm_.isStopped())
                {
                	arm_comm_.stopAPI();
                	arm_comm_.startAPI();
		        }
                //why preemted, if the robot is stalled, trajectory/action failed!
                /*
                action_server_.setPreempted(result);
                ROS_WARN_STREAM(__PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ", setPreempted ");
                */
                goal_handle->abort(result);
                RCLCPP_WARN_STREAM(rclcpp::get_logger("kinova_joint_angles_action"), __PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ", Trajectory command failed ");
                action_is_over = true;
            }

            rclcpp::WallRate(rate_hz_).sleep();
        }
    }
    catch(const std::exception& e)
    {
        result->angles = current_joint_angles.constructAnglesMsg();
        RCLCPP_WARN_STREAM(rclcpp::get_logger("kinova_joint_angles_action"), e.what());
        goal_handle->abort(result);
        RCLCPP_WARN_STREAM(rclcpp::get_logger("kinova_joint_angles_action"), __PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ", setAborted ");
    }

    // Put back the arm in Cartesian position mode
    KinovaPose pose;
    arm_comm_.getCartesianPosition(pose);
    arm_comm_.setCartesianPosition(pose, 0, false);
}

}  // namespace kinova
