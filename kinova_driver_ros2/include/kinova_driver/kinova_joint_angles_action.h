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
 *  File: kinova_joint_angles_action.h
 *  Desc: Action server for kinova arm.
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

#ifndef KINOVA_DRIVER_KINOVA_ANGLES_ACTION_H
#define KINOVA_DRIVER_KINOVA_ANGLES_ACTION_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <thread>

#include <kinova_msgs_ros2/action/arm_joint_angles.hpp>

#include "kinova_driver/kinova_comm.h"

using std::placeholders::_1;  // this is uesed in Subscriptor
using std::placeholders::_2;  // this is uesed in Subscriptor


namespace kinova
{

class KinovaAnglesActionServer
{
 public:
    KinovaAnglesActionServer(KinovaComm &, const std::shared_ptr<rclcpp::Node> &nh);
    ~KinovaAnglesActionServer();

    using Kinova_Action = kinova_msgs_ros2::action::ArmJointAngles;
    using GoalHandle_Kinova_Action = rclcpp_action::ServerGoalHandle<Kinova_Action>;

    //void actionCallback(const kinova_msgs_ros2::action::ArmPoseGoalConstPtr &);
    void execute(const std::shared_ptr<GoalHandle_Kinova_Action> goal_handle);

    // ** A callback function for handling goals
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const Kinova_Action::Goal> goal)
    {
        RCLCPP_INFO(rclcpp::get_logger("kinova"), "Received goal request at joints [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f]", 
                goal->angles.joint1, goal->angles.joint2, goal->angles.joint3, goal->angles.joint4, goal->angles.joint5, goal->angles.joint6, goal->angles.joint7);
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // ** A callback function for handling cancellation
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandle_Kinova_Action> goal_handle)
    {
        RCLCPP_INFO(rclcpp::get_logger("kinova"), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // ** A callback function for handling goal accept
    void handle_accepted(const std::shared_ptr<GoalHandle_Kinova_Action> goal_handle)
    {
        using namespace std::placeholders;
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&KinovaAnglesActionServer::execute, this, _1), goal_handle}.detach();
    }

 private:
    std::shared_ptr<rclcpp::Node> node_handle_;
    KinovaComm &arm_comm_;
    rclcpp_action::Server<kinova_msgs_ros2::action::ArmJointAngles>::SharedPtr action_server_;

    rclcpp::Time last_nonstall_time_;
    KinovaAngles last_nonstall_angles_;

    // Parameters
    double stall_interval_seconds_;
    double stall_threshold_;
    double rate_hz_;
    float tolerance_;
    double jointSpeedLimitJoints123;
    double jointSpeedLimitJoints456;
};

}  // namespace kinova
#endif  // KINOVA_DRIVER_KINOVA_ANGLES_ACTION_H
