#include "rover_comms_listener.hpp"
#include "rover_comms_serial.hpp"

void signalHandler(int signum)
{
    std::cout << "\nInterrupt signal (" << signum << ") received. Exiting..." << std::endl;

    SerialStop();
}

int main(int argc, char **argv)
{
    std::signal(SIGINT, signalHandler);

    rclcpp::init(argc, argv);
    auto                                rover_node      = std::make_shared<RoverComm>();
    std::shared_ptr<RoverCommsListener> listenCallbacks = std::make_shared<RoverCommsListener>(rover_node);

    SerialStart("/dev/ttyUSB0", 1e6U);

    rover_node->talker   = std::make_shared<cave_talk::Talker>(SerialSend);
    rover_node->listener = std::make_shared<cave_talk::Listener>(SerialReceive, listenCallbacks);

    rclcpp::spin(rover_node);
    rclcpp::shutdown();
    return 0;
}