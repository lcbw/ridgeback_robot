/**
 *
 *  \file
 *  \brief      Cooling control class for Ridgeback
 *  \author     Tony Baltovski <tbaltovski@clearpathrobotics.com>
 *  \copyright  Copyright (c) 2015, Clearpath Robotics, Inc.
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
 * Please send comments, questions, or patches to code@clearpathrobotics.com
 *
 */

#ifndef RIDGEBACK_BASE_RIDGEBACK_COOLING_H
#define RIDGEBACK_BASE_RIDGEBACK_COOLING_H

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ridgeback_msgs/msg/fans.hpp>
#include <ridgeback_msgs/msg/status.hpp>

namespace ridgeback_base
{

class RidgebackCooling
{
public:
    explicit RidgebackCooling(std::shared_ptr<rclcpp::Node> nh);

private:
    std::shared_ptr<rclcpp::Node> nh_;

    rclcpp::Publisher<ridgeback_msgs::msg::Fans>::SharedPtr cmd_fans_pub_;

    rclcpp::Subscription<ridgeback_msgs::msg::Status>::SharedPtr status_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    rclcpp::TimerBase::SharedPtr cmd_fans_timer_;

    bool charger_disconnected_;
    ridgeback_msgs::msg::Fans cmd_fans_msg_;
    rclcpp::Time last_motion_cmd_time_;

    static constexpr auto LINEAR_VEL_THRESHOLD = 0.1;   // m/s
    static constexpr auto ANGULAR_VEL_THRESHOLD = 0.4;  // rad/s
    static constexpr auto MOITON_COMMAND_TIMEOUT = 3.0; // seconds

    void statusCallback(const ridgeback_msgs::msg::Status::SharedPtr status);
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr twist);
    void cmdFansCallback();
};

}  // namespace ridgeback_base

#endif  // RIDGEBACK_BASE_RIDGEBACK_COOLING_H
