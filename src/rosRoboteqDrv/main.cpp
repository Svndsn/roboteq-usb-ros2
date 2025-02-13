#include <boost/thread.hpp>

#include "rosRoboteqDrv.h"

using namespace oxoocoffee;

const int MAX_RETRY = 5;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<RosRoboteqDrv>();

    if (!node->Initialize())
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to initialize RosRoboteqDrv");
        return -1;
    }

    node->Run();

    node->Shutdown();

    return 0;
}
