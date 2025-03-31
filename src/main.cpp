#include "rover_comms_listener.hpp"
#include "rover_comms_serial.hpp"

static std::string config_file = "";
static std::string port        = "/dev/ttyUSB0";
static uint32_t    baud        = 1000000U;

static void SignalHandler(int signum);
static bool GetSerialConfig(std::string file);

int main(int argc, char **argv)
{
    std::signal(SIGINT, SignalHandler);

    if (!GetSerialConfig(config_file))
    {
        std::cerr << "Error configuring serial" << std::endl;
    }
    else
    {
        rover_comms_serial::Start(port, baud);

        rclcpp::init(argc, argv);
        auto                                rover_node      = std::make_shared<RoverComm>();
        std::shared_ptr<RoverCommsListener> listenCallbacks = std::make_shared<RoverCommsListener>(rover_node);
        rover_node->talker   = std::make_shared<cave_talk::Talker>(rover_comms_serial::Send);
        rover_node->listener = std::make_shared<cave_talk::Listener>(rover_comms_serial::Receive, listenCallbacks);

        rclcpp::spin(rover_node);
        rclcpp::shutdown();
    }

    return 0;
}

static void SignalHandler(int signum)
{
    if ((SIGINT == signum) || (SIGTERM == signum))
    {
        std::cout << "\nInterrupt signal (" << signum << ") received. Exiting..." << std::endl;

        rover_comms_serial::Stop();
    }
}

static bool GetSerialConfig(std::string file)
{
    tinyxml2::XMLDocument doc;
    tinyxml2::XMLError    error = doc.LoadFile(file.c_str());

    if (error != tinyxml2::XML_SUCCESS)
    {
        std::cout << "Serial Config error: " << file.c_str() << std::endl;
        return false;
    }

    tinyxml2::XMLElement *root = doc.FirstChildElement("Serial_Config");
    if (!root)
    {
        std::cerr << "Missing <Serial_Config> root element" << std::endl;
        return false;
    }

    tinyxml2::XMLElement *portElem = root->FirstChildElement("port");
    if (portElem && portElem->GetText())
    {
        port = portElem->GetText();
    }
    else
    {
        std::cerr << "Using default port: " << port << std::endl;
    }

    tinyxml2::XMLElement *baudElem = root->FirstChildElement("baud");
    if (baudElem && baudElem->GetText())
    {
        baud = std::stoul(baudElem->GetText());
    }
    else
    {
        std::cerr << "Using default baud: " << baud << std::endl;
    }

    return true;
}