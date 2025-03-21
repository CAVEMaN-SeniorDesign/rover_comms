#include "rover_comms_listener.hpp"

std::shared_ptr<RoverComm> global_rover_node = nullptr; // making global so that sigHandler can access

void signalHandler(int signum){
    std::cout << "\nInterrupt signal (" << signum << ") received. Exiting..." << std::endl;
    exit(signum);
}

int main(int argc, char **argv) {
    std::signal(SIGINT, signalHandler);

    rclcpp::init(argc, argv);
    auto rover_node = std::make_shared<RoverComm>();

    std::shared_ptr<RoverCommsListener> listenCallbacks = std::make_shared<RoverCommsListener>(rover_node);

    rover_node->talker = std::make_shared<cave_talk::Talker>(cave_talk::send);
    rover_node->listener = std::make_shared<cave_talk::Listener>(cave_talk::receive, listenCallbacks);
    
    rclcpp::on_shutdown([rover_node]() {
        rover_node->talker->SpeakOogaBooga(cave_talk::SAY_BOOGA);
        cave_talk::flush();
        signalHandler(SIGINT);
    });

    rclcpp::spin(rover_node);
    rclcpp::shutdown();
    return 0;
}