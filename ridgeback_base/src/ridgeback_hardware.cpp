/**
 *
 *  \file
 *  \brief      Class representing Ridgeback hardware
 *  \author     Mike Purvis <mpurvis@clearpathrobotics.com>
 *  \author     Tony Baltovski <tbaltovski@clearpathrobotics.com>
 *  \copyright  Copyright (c) 2013, Clearpath Robotics, Inc.
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

#include <boost/asio.hpp>
#include <boost/assign.hpp>
#include <boost/thread.hpp>

#include "ridgeback_base/ridgeback_hardware.h"
#include <hardware_interface/actuator_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <puma_motor_driver/socketcan_gateway.hpp>

#include <rclcpp/logging.hpp>
#include <vector>

namespace ridgeback_base {

hardware_interface::return_type RidgebackHardware::configure(
    const hardware_interface::HardwareInfo &info)
{
    active_ = false;

    if (configure_default(info) != hardware_interface::return_type::OK) {
        return hardware_interface::return_type::ERROR;
    }

    param_listener_ = std::make_shared<ridgeback_base::ParamListener>(nh_);
    params_ = param_listener_->get_params();

    // Create gateway for the motor drivers
    std::string canbus_dev;
    canbus_dev = info_.hardware_parameters["canbus_dev"];
    //    canbus_dev = params_.canbus_dev;

    gateway_ = std::shared_ptr<puma_motor_driver::SocketCANGateway>(
        new puma_motor_driver::SocketCANGateway(canbus_dev));

    // Read all the parameters for the robot
    //    gear_ratio_ = stod(info_.hardware_parameters["gear_ratio"]);
    gear_ratio_ = params_.gear_ratio;
    //    encoder_cpr_ = stoi(info_.hardware_parameters["encoder_cpr"]);
    encoder_cpr_ = params_.encoder_cpr;

    //    flip_motor_direction_ = stoi(info_.hardware_parameters["flip_motor_direction"]);
    flip_motor_direction_ = params_.flip_motor_direction;

    //    gain_p_ = stod(info_.hardware_parameters["gains_p"]);
    gain_p_ = params_.gains_p;
    //    gain_i_ = stod(info_.hardware_parameters["gains_i"]);
    gain_i_ = params_.gains_i;
    //    gain_d_ = stod(info_.hardware_parameters["gains_d"]);
    gain_d_ = params_.gains_d;

    for (const hardware_interface::ComponentInfo &joint : info_.joints) {
        // DiffBotSystem has exactly two states and one command interface on each joint
        if (joint.command_interfaces.size() != 1) {
            RCLCPP_FATAL(rclcpp::get_logger("controller_manager"),
                         "Joint '%s' has %d command interfaces found. 1 expected.",
                         joint.name.c_str(),
                         joint.command_interfaces.size());
            return hardware_interface::return_type::ERROR;
        }

        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
            RCLCPP_FATAL(rclcpp::get_logger("controller_manager"),
                         "Joint '%s' have %s command interfaces found. '%s' expected.",
                         joint.name.c_str(),
                         joint.command_interfaces[0].name.c_str(),
                         hardware_interface::HW_IF_VELOCITY);
            return hardware_interface::return_type::ERROR;
        }

        if (joint.state_interfaces.size() != 2) {
            RCLCPP_FATAL(rclcpp::get_logger("controller_manager"),
                         "Joint '%s' has %d state interface. 2 expected.",
                         joint.name.c_str(),
                         joint.state_interfaces.size());
            return hardware_interface::return_type::ERROR;
        }

        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(rclcpp::get_logger("controller_manager"),
                         "Joint '%s' have '%s' as first state interface. '%s' and '%s' expected.",
                         joint.name.c_str(),
                         joint.state_interfaces[0].name.c_str(),
                         hardware_interface::HW_IF_POSITION);
            return hardware_interface::return_type::ERROR;
        }

        if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
            RCLCPP_FATAL(rclcpp::get_logger("controller_manager"),
                         "Joint '%s' have '%s' as second state interface. '%s' expected.",
                         joint.name.c_str(),
                         joint.state_interfaces[1].name.c_str(),
                         hardware_interface::HW_IF_VELOCITY);
            return hardware_interface::return_type::ERROR;
        }
    }

    std::vector<uint8_t> joint_can_ids;
    std::vector<float> joint_directions;
    if (info_.joints.size() == 2) {
        joint_can_ids.assign({5, 4});     // NOLINT(whitespace/braces)
        joint_directions.assign({-1, 1}); // NOLINT(whitespace/braces)
    } else if (info_.joints.size() == 4) {
        joint_can_ids.assign({5, 4, 2, 3});      // NOLINT(whitespace/braces)
        joint_directions.assign({-1, 1, -1, 1}); // NOLINT(whitespace/braces)
    } else {
        RCLCPP_FATAL(rclcpp::get_logger("controller_manager"),
                     "Unknown number of joints. Expected 2 or 4, got %d",
                     info_.joints.size());
    }

    // Flip the motor direction if needed
    if (flip_motor_direction_) {
        for (std::size_t i = 0; i < joint_directions.size(); i++) {
            joint_directions[i] *= -1;
        }
    }

    for (uint8_t i = 0; i < info_.joints.size(); i++) {
        puma_motor_driver::Driver driver(gateway_, joint_can_ids[i], info_.joints[i].name);
        driver.clearMsgCache();
        driver.setEncoderCPR(encoder_cpr_);
        driver.setGearRatio(gear_ratio_ * joint_directions[i]);
        driver.setMode(puma_motor_msgs::msg::Status::MODE_SPEED, gain_p_, gain_i_, gain_d_);
        drivers_.push_back(driver);
    }

    multi_driver_node_.reset(new puma_motor_driver::MultiDriverNode("multi_driver_node", drivers_));

    for (auto &driver : drivers_) {
        driver.configureParams();
    }

    boost::shared_ptr<boost::thread> thread(
        new boost::thread(&RidgebackHardware::canReadThread, this));

    status_ = hardware_interface::status::CONFIGURED;
    return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> RidgebackHardware::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (auto i = 0u; i < info_.joints.size(); i++) {
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(info_.joints[i].name,
                                               hardware_interface::HW_IF_POSITION,
                                               &joints_[i].position));
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(info_.joints[i].name,
                                               hardware_interface::HW_IF_VELOCITY,
                                               &joints_[i].velocity));
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RidgebackHardware::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (auto i = 0u; i < info_.joints.size(); i++) {
        command_interfaces.emplace_back(
            hardware_interface::CommandInterface(info_.joints[i].name,
                                                 hardware_interface::HW_IF_VELOCITY,
                                                 &joints_[i].velocity_command));
    }

    return command_interfaces;
}

hardware_interface::return_type RidgebackHardware::start()
{
    RCLCPP_INFO(rclcpp::get_logger("controller_manager"), "Starting ...please wait...");
    while (!connectIfNotConnected()) {
        rclcpp::Rate(1).sleep();
    }

    this->verify();

    status_ = hardware_interface::status::STARTED;
    RCLCPP_INFO(rclcpp::get_logger("controller_manager"), "System Successfully started!");
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type RidgebackHardware::stop()
{
    RCLCPP_INFO(rclcpp::get_logger("controller_manager"), "Stopping ...please wait...");

    status_ = hardware_interface::status::STOPPED;

    RCLCPP_INFO(rclcpp::get_logger("controller_manager"), "System successfully stopped!");

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type RidgebackHardware::read()
{
    if (this->isActive()) {
        this->powerHasNotReset();
        this->updateJointsFromHardware();
    } else {
        for (auto &driver : drivers_) {
            driver.configureParams();
        }
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type RidgebackHardware::write()
{
    if (this->isActive()) {
        this->command();
        this->requestData();
    } else {
        this->verify();
    }
    return hardware_interface::return_type::OK;
}

/**
 * Populates the internal joint state struct from the most recent CAN data
 * received from the motor controller.
 *
 * Called from the controller thread.
 */
void RidgebackHardware::updateJointsFromHardware() //doesn't need to be ported
{
    uint8_t index = 0;
    for (auto &driver : drivers_) {
        Joint *f = &joints_[index];
        f->effort = driver.lastCurrent();
        f->position = driver.lastPosition();
        f->velocity = driver.lastSpeed();
        index++;
    }
}

bool RidgebackHardware::areAllDriversActive() //doesn't need to be ported
{
  for (auto& driver : drivers_)
  {
    if (!driver.isConfigured())
    {
      return false;
    }
  }
  return true;
}

bool RidgebackHardware::isActive() //ported
{
    if (!active_ && this->areAllDriversActive()) {
        active_ = true;
        multi_driver_node_->activePublishers(active_);
        RCLCPP_INFO(rclcpp::get_logger("controller_manager"), "Ridgeback Hardware: Active.");
    } else if (active_ && !this->areAllDriversActive()) {
        active_ = false;
        RCLCPP_WARN(rclcpp::get_logger("controller_manager"), "Ridgeback Hardware: Inactive.");
    }

    if (!active_) {
        RCLCPP_WARN(rclcpp::get_logger("controller_manager"), "Ridgeback Hardware: Inactive");
    }
    return active_;
}

void RidgebackHardware::requestData() //doesn't need to be ported
{
  for (auto& driver : drivers_)
  {
    driver.requestFeedbackPowerState();
  }
}
void RidgebackHardware::powerHasNotReset()
{
  // Checks to see if power flag has been reset for each driver
  for (auto& driver : drivers_)
  {
    if (driver.lastPower() != 0)
    {
      active_ = false;
      RCLCPP_WARN(rclcpp::get_logger("controller_manager"),
                  "Ridgeback Hardware: Power rest on puma motor controller located on the %s(%d), "
                  "will reconfigure all drivers.",
                  driver.deviceName().c_str(),
                  driver.deviceNumber());
      multi_driver_node_->activePublishers(active_);
      for (auto& driver : drivers_)
      {
        driver.resetConfiguration();
      }
    }
  }
}

void RidgebackHardware::configure()
{
  for (auto& driver : drivers_)
  {
    driver.configureParams();
  }
}
void RidgebackHardware::verify()
{
  for (auto& driver : drivers_)
  {
    driver.verifyParams();
  }
}

bool RidgebackHardware::inReset()
{
  return !active_;
}

void RidgebackHardware::canRead()
{
  // Process all received messages through the connected driver instances.
  puma_motor_driver::Message recv_msg;
  while (gateway_->recv(&recv_msg)) {
      for (auto &driver : drivers_) {
          driver.processMessage(recv_msg);
      }
  }
}

void RidgebackHardware::canReadThread()
{
    rclcpp::Rate rate(200);
    while (rclcpp::ok()) {
        this->canRead();
        rate.sleep();
    }
}

bool RidgebackHardware::connectIfNotConnected()
{
    if (!gateway_->isConnected()) {
        if (!gateway_->connect()) {
            RCLCPP_ERROR(
                rclcpp::get_logger("my_logger"),
                "Ridgeback Hardware: Error connecting to motor driver gateway. Retrying in "
                "1 second.");
            return false;
        } else {
            RCLCPP_INFO(rclcpp::get_logger("my_logger"),
                        "Ridgeback Hardware: Connection to motor driver gateway successful.");
        }
    }
    return true;
}

/**
 * Populates and publishes Drive message based on the controller outputs.
 *
 * Called from the controller thread.
 */
void RidgebackHardware::command()
{
  uint8_t i = 0;
  for (auto& driver : drivers_)
  {
    driver.commandSpeed(joints_[i].velocity_command);
    i++;
  }
}

std::vector<puma_motor_driver::Driver>& RidgebackHardware::getDrivers()
{
  return drivers_;
}

} // namespace ridgeback_base
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ridgeback_base::RidgebackHardware, hardware_interface::SystemInterface)
