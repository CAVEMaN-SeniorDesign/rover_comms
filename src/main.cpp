#include "rover_comms_listener.hpp"
#include "rover_comms_serial.hpp"

void signalHandler(int signum)
{
    std::cout << "\nInterrupt signal (" << signum << ") received. Exiting..." << std::endl;

    rover_comms_serial::Stop();
}

int main(int argc, char **argv)
{
    std::signal(SIGINT, signalHandler);

    rover_comms_serial::Start("/dev/ttyUSB0", 1000000U);

    rclcpp::init(argc, argv);
    auto                                rover_node      = std::make_shared<RoverComm>();
    std::shared_ptr<RoverCommsListener> listenCallbacks = std::make_shared<RoverCommsListener>(rover_node);
    rover_node->talker   = std::make_shared<cave_talk::Talker>(rover_comms_serial::Send);
    rover_node->listener = std::make_shared<cave_talk::Listener>(rover_comms_serial::Receive, listenCallbacks);

    rclcpp::spin(rover_node);
    rclcpp::shutdown();
    return 0;
}