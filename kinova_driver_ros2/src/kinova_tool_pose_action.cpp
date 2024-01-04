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
 *  File: kinova_tool_pose_action.cpp
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

#include "kinova_driver/kinova_tool_pose_action.h"
#include "kinova_driver/kinova_ros_types.h"
#include <string>

void transformPose(std::unique_ptr<tf2_ros::Buffer> & tf_buffer, 
                std::string target_frame, geometry_msgs::msg::PoseStamped pose_in, geometry_msgs::msg::PoseStamped pose_out){
    auto transformStamped = tf_buffer->lookupTransform(target_frame, pose_in.header.frame_id, tf2::TimePointZero);
    tf2::doTransform(pose_in, pose_out, transformStamped); 
}

namespace kinova
{

KinovaPoseActionServer::KinovaPoseActionServer(KinovaComm &arm_comm, const std::shared_ptr<rclcpp::Node> &nh, const std::string &kinova_robotType, const std::string &kinova_robotName)
    : arm_comm_(arm_comm),
      node_handle_(nh),
      kinova_robotType_(kinova_robotType),
      kinova_robotName_(kinova_robotName)
{
    stall_interval_seconds_ = 1.0;
    stall_threshold_ = 0.005;
    rate_hz_ = 10.0;
    double position_tolerance = 0.01;
    double EulerAngle_tolerance = 2.0*M_PI/180;
    if(!node_handle_->has_parameter("stall_interval_seconds")){
        node_handle_->declare_parameter("stall_interval_seconds", stall_interval_seconds_);
    }
    if(!node_handle_->has_parameter("stall_threshold")){
        node_handle_->declare_parameter("stall_threshold", stall_threshold_);
    }
    if(!node_handle_->has_parameter("rate_hz")){
        node_handle_->declare_parameter("rate_hz", rate_hz_);
    }
    if(!node_handle_->has_parameter("position_tolerance")){
        node_handle_->declare_parameter("position_tolerance", position_tolerance);
    }
    if(!node_handle_->has_parameter("EulerAngle_tolerance")){
        node_handle_->declare_parameter("EulerAngle_tolerance", EulerAngle_tolerance);
    }
    node_handle_->get_parameter("stall_interval_seconds", stall_interval_seconds_);
    node_handle_->get_parameter("stall_threshold", stall_threshold_);
    node_handle_->get_parameter("rate_hz", rate_hz_);
    node_handle_->get_parameter("position_tolerance", position_tolerance);
    node_handle_->get_parameter("EulerAngle_tolerance", EulerAngle_tolerance);

    //    tf_prefix_ = kinova_robotType_ + "_" + boost::lexical_cast<string>(same_type_index); // in case of multiple same_type robots
    tf_prefix_ = kinova_robotName_ + "_";
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_handle_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    position_tolerance_ = static_cast<float>(position_tolerance);
    EulerAngle_tolerance_ = static_cast<float>(EulerAngle_tolerance);
    std::stringstream ss;
    ss << tf_prefix_ << "link_base";
    link_base_frame_ = ss.str();

    std::string name_ = kinova_robotType + "_driver" + "/";
    this->action_server_ = rclcpp_action::create_server<kinova_msgs_ros2::action::ArmPose>(node_handle_, name_ + "tool_pose",
                        std::bind(&KinovaPoseActionServer::handle_goal, this, _1, _2),
                        std::bind(&KinovaPoseActionServer::handle_cancel, this, _1),
                        std::bind(&KinovaPoseActionServer::handle_accepted, this, _1));
}


KinovaPoseActionServer::~KinovaPoseActionServer()
{
    RCLCPP_WARN(rclcpp::get_logger("kinova_tool_pose_action"), "destruction entered!");
}


void KinovaPoseActionServer::execute(const std::shared_ptr<GoalHandle_Kinova_Action> goal_handle)
{
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Kinova_Action::Feedback>();
    auto result = std::make_shared<Kinova_Action::Result>();
    feedback->pose.header.frame_id = goal->pose.header.frame_id;
    result->pose.header.frame_id = goal->pose.header.frame_id;

    rclcpp::Time current_time = rclcpp::Clock{}.now();
    KinovaPose current_pose;
    geometry_msgs::msg::PoseStamped local_pose;
    local_pose.header.frame_id = link_base_frame_;

    try
    {
        // Put the goal pose into the frame used by the arm
        if (rclcpp::ok()
                && !tf_buffer_->canTransform(link_base_frame_, goal->pose.header.frame_id,
                                          goal->pose.header.stamp))
        {
            RCLCPP_ERROR(rclcpp::get_logger("kinova_tool_pose_action"), "Could not get transfrom from %s to %s, aborting cartesian movement",
                      link_base_frame_.c_str(), goal->pose.header.frame_id.c_str());
            goal_handle->abort(result);
            RCLCPP_WARN_STREAM(rclcpp::get_logger("kinova_tool_pose_action"), __PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ", setAborted ");
            return;
        }

        transformPose(tf_buffer_, local_pose.header.frame_id, goal->pose, local_pose);
        //tf_listener_.transformPose(local_pose.header.frame_id, goal->pose, local_pose);
        arm_comm_.getCartesianPosition(current_pose);
        if (arm_comm_.isStopped())
        {
            RCLCPP_INFO(rclcpp::get_logger("kinova_tool_pose_action"), "Could not complete cartesian action because the arm is 'stopped'.");
            local_pose.pose = current_pose.constructPoseMsg();
            transformPose(tf_buffer_, result->pose.header.frame_id, local_pose, result->pose);
            //tf_listener_.transformPose(result.pose.header.frame_id, local_pose, result.pose);
            goal_handle->abort(result);
            RCLCPP_WARN_STREAM(rclcpp::get_logger("kinova_tool_pose_action"), __PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ", setAborted ");
            return;
        }

        last_nonstall_time_ = current_time;
        last_nonstall_pose_ = current_pose;

        KinovaPose target(local_pose.pose);
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("kinova_tool_pose_action"), std::endl << std::endl << "***-----------------------***" << std::endl << __PRETTY_FUNCTION__ << ":  target X " << target.X << "; Y "<< target.Y << "; Z "<< target.Z << "; ThetaX " << target.ThetaX << "; ThetaY " << target.ThetaY  << "; ThetaZ " << target.ThetaZ << std::endl << "***-----------------------***" << std::endl );
        arm_comm_.setCartesianPosition(target);
        while (rclcpp::ok())
        {
            // without setCartesianPosition() in while loop, robot stopped in the half way, and the goal won't be reached.
            arm_comm_.setCartesianPosition(target);
            rclcpp::spin_some(node_handle_);

            if (arm_comm_.isStopped())
            {
                RCLCPP_DEBUG_STREAM(rclcpp::get_logger("kinova_tool_pose_action"), "" << __PRETTY_FUNCTION__ << ": arm_comm_.isStopped()");
                result->pose = feedback->pose;
                goal_handle->abort(result);
                RCLCPP_WARN_STREAM(rclcpp::get_logger("kinova_tool_pose_action"), __PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ", setAborted ");
                return;
            }
            else if (goal_handle->is_canceling() || !rclcpp::ok())
            {
                RCLCPP_DEBUG_STREAM(rclcpp::get_logger("kinova_tool_pose_action"), "" << __PRETTY_FUNCTION__ << ": action server isPreemptRequested");
                result->pose = feedback->pose;
                arm_comm_.stopAPI();
                arm_comm_.startAPI();
                goal_handle->canceled(result);
                RCLCPP_WARN_STREAM(rclcpp::get_logger("kinova_tool_pose_action"), __PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ", setPreempted ");
                return;
            }

            arm_comm_.getCartesianPosition(current_pose);
            current_time = rclcpp::Clock{}.now();
            local_pose.pose = current_pose.constructPoseMsg();
            transformPose(tf_buffer_, feedback->pose.header.frame_id, local_pose, feedback->pose);
            //tf_listener_.transformPose(feedback.pose.header.frame_id, local_pose, feedback.pose);
            //action_server_.publishFeedback(feedback);

            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("kinova_tool_pose_action"), "" << __PRETTY_FUNCTION__ << ": current_pose X " << current_pose.X << "; Y "<< current_pose.Y << "; Z "<< current_pose.Z << "; ThetaX " << current_pose.ThetaX << "; ThetaY " << current_pose.ThetaY  << "; ThetaZ " << current_pose.ThetaZ );

            if (target.isCloseToOther(current_pose, position_tolerance_, EulerAngle_tolerance_))
            {
                RCLCPP_DEBUG_STREAM(rclcpp::get_logger("kinova_tool_pose_action"), "" << __PRETTY_FUNCTION__ << ": arm_comm_.isCloseToOther");
                result->pose = feedback->pose;
                goal_handle->succeed(result);
                RCLCPP_WARN_STREAM(rclcpp::get_logger("kinova_tool_pose_action"), __PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ", setSucceeded ");
                return;
            }
            else if (!last_nonstall_pose_.isCloseToOther(current_pose, stall_threshold_, stall_threshold_))
            {
                // Check if we are outside of a potential stall condition
                last_nonstall_time_ = current_time;
                last_nonstall_pose_ = current_pose;
            }
            else if ((current_time - last_nonstall_time_).seconds() > stall_interval_seconds_)
            {
                RCLCPP_DEBUG_STREAM(rclcpp::get_logger("kinova_tool_pose_action"), "" << __PRETTY_FUNCTION__ << ": stall_interval_seconds_");
                // Check if the full stall condition has been meet
                result->pose = feedback->pose;
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
                RCLCPP_WARN_STREAM(rclcpp::get_logger("kinova_tool_pose_action"), __PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ", Trajectory command failed ");
                return;
            }
            rclcpp::WallRate(rate_hz_).sleep();
        }
    }
    catch(const std::exception& e)
    {
        result->pose = feedback->pose;
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("kinova_tool_pose_action"), e.what());
        goal_handle->abort(result);
        RCLCPP_WARN_STREAM(rclcpp::get_logger("kinova_tool_pose_action"), __PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ", setAborted ");
    }
}

}  // namespace kinova
