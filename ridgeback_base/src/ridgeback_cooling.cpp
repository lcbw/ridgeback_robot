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

#include <math.h>

#include "ridgeback_base/ridgeback_cooling.h"

namespace ridgeback_base {

RidgebackCooling::RidgebackCooling(std::shared_ptr<rclcpp::Node> nh)
    : nh_(nh)
    , charger_disconnected_(true)
{
    cmd_fans_pub_ = nh_->create_publisher<ridgeback_msgs::msg::Fans>("mcu/cmd_fans", 1);

    status_sub_ = nh_->create_subscription<ridgeback_msgs::msg::Status>(
        "mcu/status", 1, std::bind(&RidgebackCooling::statusCallback, this, std::placeholders::_1));

    cmd_vel_sub_ = nh_->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 1, std::bind(&RidgebackCooling::cmdVelCallback, this, std::placeholders::_1));

    cmd_fans_timer_ = nh_->create_wall_timer(std::chrono::milliseconds(100),
                                             std::bind(&RidgebackCooling::cmdFansCallback, this));

    for (std::size_t i = 0; i < 6; i++) {
        cmd_fans_msg_.fans[i] = ridgeback_msgs::msg::Fans::FAN_ON_LOW;
    }
}

void RidgebackCooling::statusCallback(const ridgeback_msgs::msg::Status::SharedPtr status)
{
  if (status->charger_connected)
  {
      cmd_fans_msg_.fans[ridgeback_msgs::msg::Fans::CHARGER_BAY_INTAKE]
          = ridgeback_msgs::msg::Fans::FAN_ON_HIGH;
      cmd_fans_msg_.fans[ridgeback_msgs::msg::Fans::CHARGER_BAY_EXHAUST]
          = ridgeback_msgs::msg::Fans::FAN_ON_HIGH;
      charger_disconnected_ = false;
  }
  else if (!charger_disconnected_)
  {
      cmd_fans_msg_.fans[ridgeback_msgs::msg::Fans::CHARGER_BAY_INTAKE]
          = ridgeback_msgs::msg::Fans::FAN_ON_LOW;
      cmd_fans_msg_.fans[ridgeback_msgs::msg::Fans::CHARGER_BAY_EXHAUST]
          = ridgeback_msgs::msg::Fans::FAN_ON_LOW;
      charger_disconnected_ = true;
  }
}
void RidgebackCooling::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr twist)
{
  if (fabs(twist->linear.x) >= LINEAR_VEL_THRESHOLD ||
      fabs(twist->linear.y) >= LINEAR_VEL_THRESHOLD ||
      fabs(twist->angular.z) >= ANGULAR_VEL_THRESHOLD)
  {
    for (std::size_t i = 0; i < 6; i++)
    {
        cmd_fans_msg_.fans[i] = ridgeback_msgs::msg::Fans::FAN_ON_HIGH;
    }
  }
  last_motion_cmd_time_ = nh_->get_clock()->now();
}

void RidgebackCooling::cmdFansCallback()
{
    if (((nh_->get_clock()->now() - last_motion_cmd_time_).seconds() > MOITON_COMMAND_TIMEOUT)
        && charger_disconnected_) {
        for (std::size_t i = 0; i < 6; i++) {
            cmd_fans_msg_.fans[i] = ridgeback_msgs::msg::Fans::FAN_ON_LOW;
        }
    }
    cmd_fans_pub_->publish(cmd_fans_msg_);
}

} // namespace ridgeback_base
