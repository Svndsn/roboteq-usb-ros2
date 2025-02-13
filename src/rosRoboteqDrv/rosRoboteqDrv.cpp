#include "rosRoboteqDrv.h"

#include "roboteq_node_ros2/msg/wheels_msg.hpp"
#include "roboteq_node_ros2/srv/actuators.hpp"
#include "roboteq_node_ros2/srv/send_can_command.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_cpp/service_type_support.hpp"

typedef std::vector<std::string> TStrVec;
void Split(TStrVec& vec, const string& str);

RosRoboteqDrv::RosRoboteqDrv(void)
    : _nh(std::make_shared<rclcpp::Node>(NODE_NAME)),
      _logEnabled(false),
      _comunicator(*this, *this)
{
}

bool RosRoboteqDrv::Initialize()
{
    try
    {
        std::string mode;

        if (!_nh->get_parameter("mode", mode))
        {
            RCLCPP_FATAL(_nh->get_logger(), "Please specify mode parameter");
            return false;
        }

        std::transform(mode.begin(), mode.end(), mode.begin(), ::tolower);

        std::string device;

        if (!_nh->get_parameter("device", device))
        {
            RCLCPP_FATAL(_nh->get_logger(), "Please specify device parameter");
            return false;
        }

        if (!_nh->get_parameter("left", _left))
        {
            RCLCPP_FATAL(_nh->get_logger(), "Please specify left parameter");
            return false;
        }

        if (!_nh->get_parameter("right", _right))
        {
            RCLCPP_FATAL(_nh->get_logger(), "Please specify right parameter");
            return false;
        }

        RCLCPP_INFO(_nh->get_logger(), "Channels Right: %s, Left: %s", _right.c_str(), _left.c_str());

        _pub = _nh->create_publisher<roboteq_node_ros2::msg::WheelsMsg>("current_velocity", 10);

        _service = _nh->create_service<roboteq_node_ros2::srv::Actuators>(
            "set_actuator_position", std::bind(&RosRoboteqDrv::SetActuatorPosition, this, std::placeholders::_1, std::placeholders::_2));
        _can_service = _nh->create_service<roboteq_node_ros2::srv::SendCANCommand>(
            "manual_can_command", std::bind(&RosRoboteqDrv::ManualCANCommand, this, std::placeholders::_1, std::placeholders::_2));

        // Do not remove below line. Else it will
        // not print diag msg from lower libs
        _logEnabled = true;

        if (mode == "can")
            _comunicator.Open(RoboteqCom::eCAN, device);
        else
            _comunicator.Open(RoboteqCom::eSerial, device);

        if (_comunicator.Version().empty())
            THROW_RUNTIME_ERROR("Failed to receive Roboteq Version");

        if (_comunicator.Model().empty())
            THROW_RUNTIME_ERROR("Failed to receive Roboteq Model");

        if (_comunicator.IsThreadRunning() == false)
            THROW_RUNTIME_ERROR("Failed to spawn RoboReader Thread");

        _comunicator.IssueCommand("# C");  // Clears out telemetry strings

        if (_comunicator.Mode() == RoboteqCom::eSerial)
        {
            _comunicator.IssueCommand("?S");     // Query for speed and enters this speed
                                                 // request into telemetry system
            _comunicator.IssueCommand("# 100");  // auto message response is 500ms
        }

        _sub = _nh->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&RosRoboteqDrv::CmdVelCallback, this, std::placeholders::_1));
    }
    catch (std::exception& ex)
    {
        RCLCPP_ERROR(_nh->get_logger(), "Open Port Failed. Error: %s", ex.what());
        throw;
    }

    return true;
}

void RosRoboteqDrv::Run(void)
{
    rclcpp::spin(_nh);
}

void RosRoboteqDrv::Shutdown(void)
{
    rclcpp::shutdown();
}

void RosRoboteqDrv::CmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr twist_velocity)
{
    _wheelVelocity = ConvertTwistToWheelVelocity(twist_velocity);

    float leftVelRPM = _wheelVelocity.left / RPM_TO_RAD_PER_SEC;
    float rightVelRPM = _wheelVelocity.right / RPM_TO_RAD_PER_SEC;

    // now round the wheel velocity to int

    std::stringstream ss;

    if (_comunicator.Mode() == RoboteqCom::eSerial)
    {
        ss << "!G " << _left << " " << (((int)leftVelRPM) * 100);
        ss << "_!G " << _right << " " << (((int)rightVelRPM) * 100);
    }
    else
    {
        // ss << "@00!G " << _left << " " << (((int)leftVelRPM) * 100);
        // ss << "_@00!G " << _right << " " << (((int)rightVelRPM) * 100);

        for (int i = 1; i <= 3; i++)  // 3 is number of wheel pairs
        {
            if (i != 1)
                ss << "_";

            ss << "@0" << i << "!G " << _left << " " << (((int)leftVelRPM) * 100);
            ss << "_@0" << i << "!G " << _right << " " << (((int)rightVelRPM) * 100);
        }
    }

    try
    {
        _comunicator.IssueCommand(ss.str());
        RCLCPP_INFO(_nh->get_logger(), "WheelsMsg= %s", ss.str().c_str());
    }
    catch (std::exception& ex)
    {
        RCLCPP_ERROR(_nh->get_logger(), "IssueCommand : %s", ex.what());
        throw;
    }
    catch (...)
    {
        RCLCPP_ERROR(_nh->get_logger(), "IssueCommand : ?");
        throw;
    }
}

bool RosRoboteqDrv::SetActuatorPosition(const std::shared_ptr<TSrvAct_Req> req,
                                        std::shared_ptr<TSrvAct_Res> res)
{
    std::stringstream ss;

    if (_comunicator.Mode() == RoboteqCom::eCAN)
    {
        ss << "@04!G 1 " << req->actuator_position;
        ss << "_@04!G 2 " << req->actuator_position;

        // ss << "@00!G 1 " << req.actuator_position;
        // ss << "_@00!G 2 " << req.actuator_position;
    }

    try
    {
        _comunicator.IssueCommand(ss.str());
        RCLCPP_INFO(_nh->get_logger(), "Actr= %s", ss.str().c_str());
    }
    catch (std::exception& ex)
    {
        RCLCPP_ERROR(_nh->get_logger(), "IssueCommand : %s", ex.what());
        throw;
    }
    catch (...)
    {
        RCLCPP_ERROR(_nh->get_logger(), "IssueCommand : ?");
        throw;
    }
    return true;
}

bool RosRoboteqDrv::ManualCANCommand(const std::shared_ptr<TSrvCAN_Req> req,
                                     std::shared_ptr<TSrvCAN_Res> res)
{
    std::stringstream ss;

    if (_comunicator.Mode() == RoboteqCom::eCAN)
    {
        ss << "@0" << req->can_id << "!G " << req->channel << " " << req->speed;
    }

    try
    {
        _comunicator.IssueCommand(ss.str());
        RCLCPP_INFO(_nh->get_logger(), "ManualCMD= %s", ss.str().c_str());
    }
    catch (std::exception& ex)
    {
        RCLCPP_ERROR(_nh->get_logger(), "IssueCommand : %s", ex.what());
        throw;
    }
    catch (...)
    {
        RCLCPP_ERROR(_nh->get_logger(), "IssueCommand : ?");
        throw;
    }
    return true;
}

geometry_msgs::msg::Twist RosRoboteqDrv::ConvertWheelVelocityToTwist(float left_velocity, float right_velocity)
{
    // using the two equations for left and right, we solve for long. vel and we get two equations for it. Add them together, and we end up with VL = (right - left) * r / 2
    float longitudinal_velocity = (right_velocity - left_velocity) * (WHEEL_DIAMETER_SCIPIO / 4);

    geometry_msgs::msg::Twist twistVelocity;

    // linear.x is just the average of left and right wheel velocities converted to linear by multiplying it by radius
    twistVelocity.linear.x = ((left_velocity + right_velocity) / 2) * (WHEEL_DIAMETER_SCIPIO / 2);

    twistVelocity.angular.z = (longitudinal_velocity * 2 * TRACK_WIDTH) / (TRACK_WIDTH * TRACK_WIDTH + WHEEL_BASE * WHEEL_BASE);

    return twistVelocity;
}

roboteq_node_ros2::msg::WheelsMsg RosRoboteqDrv::ConvertTwistToWheelVelocity(const geometry_msgs::msg::Twist::SharedPtr twist_velocity)  // look into removing permanently
{
    float longitudinalVelocity = ((twist_velocity->angular.z) * (TRACK_WIDTH * TRACK_WIDTH + WHEEL_BASE * WHEEL_BASE)) / (2 * TRACK_WIDTH);

    roboteq_node_ros2::msg::WheelsMsg wheelVelocity;

    wheelVelocity.left = -1 * longitudinalVelocity + twist_velocity->linear.x;
    wheelVelocity.right = longitudinalVelocity + twist_velocity->linear.x;

    return wheelVelocity;
}

// RoboteqCom Events
void RosRoboteqDrv::OnMsgEvent(const IEventArgs& evt)
{
    RCLCPP_DEBUG(_nh->get_logger(), "OnMsgEvent: %s", evt.Reply().c_str());

    switch (evt.Reply()[0])
    {
        case 'S':
            Process_S(evt);
            break;

        case 'G':
            Process_G(evt);
            break;

        case 'N':
            Process_N(evt);
            break;

        default:
            break;
    }
}

void RosRoboteqDrv::Process_S(const IEventArgs& evt)
{
    try
    {
        string::size_type idx = evt.Reply().find_first_of('=');

        if (idx != string::npos)
        {
            // idx++;
            string::size_type idy = evt.Reply().find_first_of(':', idx);

            if (idy != string::npos)
            {
                char* pVal1 = (char*)(evt.Reply().c_str() + idx + 1);
                char* pVal2 = (char*)(evt.Reply().c_str() + idy);

                *pVal2 = 0L;
                pVal2++;

                int firstVal = atoi(pVal1);
                int secondVal = atoi(pVal2);

                roboteq_node_ros2::msg::WheelsMsg wheelVelocity;

                wheelVelocity.right = firstVal * RPM_TO_RAD_PER_SEC;
                wheelVelocity.left = secondVal * RPM_TO_RAD_PER_SEC;

                _pub->publish(wheelVelocity);
                // RCLCPP_INFO(_nh->get_logger(), "Wheel RPM's: %d :: %d", firstVal, secondVal);
            }
            else
            {
                RCLCPP_ERROR(_nh->get_logger(), "Invalid(2) S Reply Format");
            }
        }
        else
        {
            RCLCPP_ERROR(_nh->get_logger(), "Invalid(1) S Reply Format");
        }
    }
    catch (std::exception& ex)
    {
        RCLCPP_ERROR(_nh->get_logger(), "Process_S : %s", ex.what());
    }
    catch (...)
    {
        RCLCPP_ERROR(_nh->get_logger(), "Process_S : ?");
    }
}

void RosRoboteqDrv::Process_G(const IEventArgs& evt)
{
}

void RosRoboteqDrv::Process_N(const IEventArgs& evt)
{
}

bool RosRoboteqDrv::IsLogOpen(void) const
{
    return _logEnabled;
}

// RoboteqCom and app Log Messages. Do append newline
void RosRoboteqDrv::LogLine(const char* pBuffer, unsigned int len)
{
    RCLCPP_INFO(_nh->get_logger(), " - %s", pBuffer);
}

// RoboteqCom and app Log Messages. Do append newline
void RosRoboteqDrv::LogLine(const std::string& message)
{
    RCLCPP_INFO(_nh->get_logger(), " - %s", message.c_str());
}

// RoboteqCom and app Log Messages. Do not append newline
void RosRoboteqDrv::Log(const char* pBuffer, unsigned int len)
{
    RCLCPP_INFO(_nh->get_logger(), " - %s", pBuffer);
}

// RoboteqCom and app Log Messages. Do not append newline
void RosRoboteqDrv::Log(const std::string& message)
{
    RCLCPP_INFO(_nh->get_logger(), "%s", message.c_str());
}

void Split(TStrVec& vec, const string& str)
{
    if (str.empty())
        return;

    string::size_type startIdx(0);

    while (true)
    {
        string::size_type startIdy = str.find_first_of(' ', startIdx);

        if (startIdy == string::npos)
        {
            vec.push_back(str.substr(startIdx));
            break;
        }
        else
        {
            vec.push_back(str.substr(startIdx, startIdy - startIdx));
            startIdx = startIdy + 1;
        }
    }
}
