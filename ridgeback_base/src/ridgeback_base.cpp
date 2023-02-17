/**
 *
 *  \file
 *  \brief      Main entry point for ridgeback base.
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

// this is the only thing I need to port hooray!! //
//#include <boost/asio.hpp>
#include <boost/bind.hpp>
//#include <boost/thread.hpp>
#include <chrono>  // NOLINT(build/c++11)
#include <memory>
#include <string>
#include <thread>  // NOLINT(build/c++11)

//#include "rosserial_server/udp_socket_session.h"
#include "ridgeback_base/passive_joint_publisher.h"
#include "ridgeback_base/ridgeback_cooling.h"
#include "ridgeback_base/ridgeback_diagnostic_updater.h"
#include "ridgeback_base/ridgeback_hardware.h"
#include "ridgeback_base/ridgeback_lighting.h"
#include <controller_interface/controller_interface.hpp>
#include <controller_manager/controller_manager.hpp>
#include <puma_motor_driver/diagnostic_updater.hpp>
#include <puma_motor_driver/socketcan_gateway.hpp>
#include <rclcpp/rclcpp.hpp>

//using boost::asio::ip::udp;
//using boost::asio::ip::address;

void controlThread(double rate,
                   ridgeback_base::RidgebackHardware &robot,
                   controller_manager::ControllerManager &cm)
{
  std::chrono::steady_clock::time_point last_time = std::chrono::steady_clock::now();

  while (rclcpp::ok()) {
      // Calculate monotonic time elapsed
      std::chrono::steady_clock::time_point this_time = std::chrono::steady_clock::now();
      std::chrono::duration<double> elapsed_duration = this_time - last_time;
      rclcpp::Duration elapsed(elapsed_duration.count());
      last_time = this_time;

      if (robot.isActive()) {
          robot.powerHasNotReset();
          robot.updateJointsFromHardware();
      } else {
          robot.configure();
      }

      cm.update();

      if (robot.isActive()) {
          robot.command();
          robot.requestData();
      } else {
          robot.verify();
      }

      rclcpp::Rate(rate).sleep();
  }
}

void canReadThread(double rate, ridgeback_base::RidgebackHardware &robot)
{
    while (rclcpp::ok()) {
        robot.canRead();
        rclcpp::Rate(rate).sleep();
    }
}

int main(int argc, char* argv[])
{
  // Constants
  static const std::string NODE_NAME = "ridgeback_node";
  static constexpr auto BASE_IP = "192.168.131.1";
  static constexpr auto MCU_IP = "192.168.131.2";
  static constexpr auto ROSSERIAL_PORT = 11411;
  //  static constexpr auto CAN_READ_THREAD_RATE = 200.0;
  //  static constexpr auto CONTROL_THREAD_RATE = 25.0;
  static constexpr auto PASSIVE_JOINT_RATE = 50.0;
  static const std::vector<std::string> PASSIVE_JOINT_NAMES = {("front_rocker")};

  double CAN_READ_THREAD_RATE = 200.0;
  double CONTROL_THREAD_RATE = 25.0;
  // Initialize ROS node.
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> nh;

  // ROS parameters
  std::string canbus_dev = "canbus_dev";
  std::string gateway_name = "can0";
  //  nh.param<std::string>("canbus_dev", canbus_dev, "can0");
  //  bool use_mcu = true;
  //  pnh.param<bool>("use_mcu", use_mcu, true);

  // Create the socket rosserial server in a background ASIO event loop.
  //  boost::asio::io_service io_service;
  //  std::shared_ptr<rosserial_server::UdpSocketSession> socket(nullptr);
  //  std::thread socket_thread;

  //  if (use_mcu)
  //  {
  //    socket = std::make_shared<rosserial_server::UdpSocketSession>(io_service,
  //        udp::endpoint(address::from_string(BASE_IP), ROSSERIAL_PORT),
  //        udp::endpoint(address::from_string(MCU_IP), ROSSERIAL_PORT));
  //    socket_thread = boost::thread(boost::bind(&boost::asio::io_service::run, &io_service));
  //  }

  std::shared_ptr<puma_motor_driver::SocketCANGateway> gateway
      = std::make_shared<puma_motor_driver::SocketCANGateway>(gateway_name);
  std::shared_ptr<ridgeback_base::RidgebackHardware> ridgeback
      = std::make_shared<ridgeback_base::RidgebackHardware>();

  // Configure the CAN connection
  ridgeback->start();
  // Create a thread to start reading can messages.

  // looks like std::thread and std::bind are incompatible because bind will make things into ptrs
  std::thread can_read_thread(canReadThread, CAN_READ_THREAD_RATE, std::ref(*ridgeback));

  std::shared_ptr<rclcpp::Executor> executor
      = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  std::string manager_node_name = "controller_manager";

  std::shared_ptr<controller_manager::ControllerManager> cm
      = std::make_shared<controller_manager::ControllerManager>(executor, manager_node_name);

  std::thread control_thread(controlThread,
                             CONTROL_THREAD_RATE,
                             std::ref(*ridgeback),
                             std::ref(*cm));

  // Create diagnostic updater, to update itself on the ROS thread.
  ridgeback_base::RidgebackDiagnosticUpdater ridgeback_diagnostic_updater(nh);
  puma_motor_driver::PumaMotorDriverDiagnosticUpdater puma_motor_driver_diagnostic_updater("nh");

  // Joint state publisher for passive front axle.
  ridgeback_base::PassiveJointPublisher passive_joint_publisher(nh, PASSIVE_JOINT_NAMES, PASSIVE_JOINT_RATE);

  // Cooling control for the fans.
  std::shared_ptr<ridgeback_base::RidgebackCooling> cooling
      = std::make_shared<ridgeback_base::RidgebackCooling>(nh); // tried &nh and that didn't work
  // Lighting control.
  std::shared_ptr<ridgeback_base::RidgebackLighting> lighting
      = std::make_shared<ridgeback_base::RidgebackLighting>(nh); // tried &nh and that didn't work

  //  if (use_mcu)
  //  {
  //  }

  // Foreground ROS spinner for ROS callbacks, including rosserial, diagnostics

  rclcpp::spin(nh);
  executor->add_node(cm);
  executor->spin();
  // End threads
  can_read_thread.join();
  control_thread.join();
  return 0;
}
