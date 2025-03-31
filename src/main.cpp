#include "rover_comms_listener.hpp"

void signalHandler(int signum)
{
    std::cout << "\nInterrupt signal (" << signum << ") received. Exiting..." << std::endl;
}

int main(int argc, char **argv)
{
    std::signal(SIGINT, signalHandler);

    rclcpp::init(argc, argv);
    auto                                rover_node      = std::make_shared<RoverComm>();
    std::shared_ptr<RoverCommsListener> listenCallbacks = std::make_shared<RoverCommsListener>(rover_node);

    rover_comms::CaveTalkSerialPort cave_talk_serial_port();

    rover_node->talker   = std::make_shared<rover_comms::TalkerWrapper>(cave_talk_serial_port);
    rover_node->listener = std::make_shared<rover_comms::ListenerWrapper>(cave_talk_serial_port, listenCallbacks);

    rclcpp::spin(rover_node);
    rclcpp::shutdown();
    return 0;
}