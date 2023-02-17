/**
Software License Agreement (BSD)

\file      ridgeback_lighting.h
\authors   Tony Baltovski <tbaltovski@clearpathrobotics.com>
\copyright Copyright (c) 2015, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#ifndef RIDGEBACK_BASE_RIDGEBACK_LIGHTING_H
#define RIDGEBACK_BASE_RIDGEBACK_LIGHTING_H

#include <boost/array.hpp>
#include <boost/assign/list_of.hpp>
#include <cstring>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <puma_motor_msgs/msg/multi_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ridgeback_msgs/msg/lights.hpp>
#include <ridgeback_msgs/msg/status.hpp>

namespace ridgeback_base
{

typedef boost::array<uint32_t, 8> pattern;
typedef std::vector<pattern> LightsPatterns;


class RidgebackLighting
{
public:
    explicit RidgebackLighting(std::shared_ptr<rclcpp::Node> nh);

private:
    std::shared_ptr<rclcpp::Node> nh_;

    rclcpp::Publisher<ridgeback_msgs::msg::Lights>::SharedPtr lights_pub_;

    rclcpp::Subscription<ridgeback_msgs::msg::Lights>::SharedPtr user_cmds_sub_;
    rclcpp::Subscription<ridgeback_msgs::msg::Status>::SharedPtr mcu_status_sub_;
    rclcpp::Subscription<puma_motor_msgs::msg::MultiStatus>::SharedPtr puma_status_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    puma_motor_msgs::msg::MultiStatus pumas_status_msg_;
    ridgeback_msgs::msg::Status mcu_status_msg_;
    geometry_msgs::msg::Twist cmd_vel_msg_;

    rclcpp::TimerBase::SharedPtr pub_timer_;
    rclcpp::TimerBase::SharedPtr user_timeout_;

    bool allow_user_;
    bool user_publishing_;
    uint8_t state_;
    uint8_t old_state_;
    uint8_t current_pattern_count_;
    uint32_t current_pattern_[8];

    struct patterns
    {
        LightsPatterns stopped;
        LightsPatterns fault;
        LightsPatterns reset;
        LightsPatterns low_battery;
        LightsPatterns charged;
        LightsPatterns charging;
        LightsPatterns driving;
        LightsPatterns idle;
    } patterns_;

    void setRGB(ridgeback_msgs::msg::RGB *rgb, uint32_t colour);
    void setLights(ridgeback_msgs::msg::Lights *lights, uint32_t pattern[8]);

    void updateState();
    void updatePattern();

    void userCmdCallback(const ridgeback_msgs::msg::Lights::SharedPtr lights_msg);
    void mcuStatusCallback(const ridgeback_msgs::msg::Status::SharedPtr status_msg);
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void pumaStatusCallback(const puma_motor_msgs::msg::MultiStatus::SharedPtr status_msg);
    //  void timerCb(const rclcpp::TimerEvent &);
    //  void userTimeoutCb(const rclcpp::TimerEvent &);
    void timerCb();
    void userTimeoutCb();
};

}  // namespace ridgeback_base

#endif  // RIDGEBACK_BASE_RIDGEBACK_LIGHTING_H
