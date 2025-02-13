#ifndef __ROBOTEQ_DRV_H__
#define __ROBOTEQ_DRV_H__

#include <geometry_msgs/msg/twist.hpp>  // Twist message file
#include <roboteq_node_ros2/msg/wheels_msg.hpp>
#include <roboteq_node_ros2/srv/actuators.hpp>
#include <roboteq_node_ros2/srv/send_can_command.hpp>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "roboteqCom.h"

/// Scipio dimensions and stuff
#define METERS_PER_TICK_SCIPIO 0.0011169116
#define WHEEL_DIAMETER_SCIPIO 0.3556
#define TICK_COLLECTION_PERIOD 0.05
#define TRACK_WIDTH 0.8636  // 34 inches
#define WHEEL_BASE 0.5334   // 21 inches
#define SLEEP_INTERVAL 0.05
#define RPM_TO_RAD_PER_SEC 0.1047

#define NODE_NAME "roboteq_node_ros2"

// ROS Roboteq Driver
// Robert J. Gebis (oxoocoffee) <rjgebis@yahoo.com>
// Krystian R. Gebis            <krgebis@gmail.com>
// EDT Chicago (UIC) 2014
//
// Version 1.0 (05/20/14) - Created Template for all functions needed
// Version 1.1 (05/28/14) - Wrote functions for basic communication
//                          using ros to physical Roboteq
// Version 1.2 (05/29/14) - Twist and Velocity conversion function work
//                          correctly. No more ros::init() error.
//
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License as
// published by the Free Software Foundation; either version 2 of
// the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Public License for more details at
// http://www.gnu.org/copyleft/gpl.html
// RMC: SDC2160N is OPEN-LOOP speed CAN mode.
// IGVC: Our Roboteq HDC2450 is in CLOSED-LOOP speed mode. Set the MXRPM
// through the Roborun/RoboteqDbg tools.

using namespace oxoocoffee;

class RosRoboteqDrv : public SerialLogger, public IEventListener<const IEventArgs>
{
    typedef roboteq_node_ros2::msg::WheelsMsg TWheelMsg;
    typedef geometry_msgs::msg::Twist TTwist;

    typedef roboteq_node_ros2::srv::Actuators::Request TSrvAct_Req;
    typedef roboteq_node_ros2::srv::Actuators::Response TSrvAct_Res;

    typedef roboteq_node_ros2::srv::SendCANCommand::Request TSrvCAN_Req;
    typedef roboteq_node_ros2::srv::SendCANCommand::Response TSrvCAN_Res;

   public:
    RosRoboteqDrv(void);

    bool Initialize(void);
    void Run(void);
    void Shutdown(void);
    void CmdVelCallback(const TTwist::SharedPtr twist_velocity);
    bool SetActuatorPosition(const std::shared_ptr<TSrvAct_Req> req,
                             std::shared_ptr<TSrvAct_Res> res);
    bool ManualCANCommand(const std::shared_ptr<TSrvCAN_Req> req,
                          std::shared_ptr<TSrvCAN_Res> res);

    static TWheelMsg ConvertTwistToWheelVelocity(const TTwist::SharedPtr twist_velocity);
    static TTwist ConvertWheelVelocityToTwist(float left_velocity, float right_velocity);

    rclcpp::Node::SharedPtr _nh;

   protected:
    // RoboteqCom Events
    virtual void OnMsgEvent(const IEventArgs& evt);

    virtual bool IsLogOpen(void) const;

    // Does write new line at the end
    virtual void LogLine(const char* pBuffer, unsigned int len);
    virtual void LogLine(const std::string& message);

    // Does NOT write new line at end
    virtual void Log(const char* pBuffer, unsigned int len);
    virtual void Log(const std::string& message);

    void Process_S(const IEventArgs& evt);
    void Process_G(const IEventArgs& evt);
    void Process_N(const IEventArgs& evt);

   private:
    bool _logEnabled;
    RoboteqCom _comunicator;
    rclcpp::Subscription<TTwist>::SharedPtr _sub;
    rclcpp::Publisher<TWheelMsg>::SharedPtr _pub;
    rclcpp::Service<roboteq_node_ros2::srv::Actuators>::SharedPtr _service;
    rclcpp::Service<roboteq_node_ros2::srv::SendCANCommand>::SharedPtr _can_service;
    TWheelMsg _wheelVelocity;
    std::string _left;
    std::string _right;
};

#endif  // __ROBOTEQ_DRV_H__
