#include "new_serial.hpp"


namespace new_serial{
    // default parameters, overwritten by the settings in src/Serial_Config.xml
    std::string port = "/dev/ttyTHS1"; 
    unsigned long baud = 1000000; 
    serial::Serial new_serial;
    serial::Timeout timeout = serial::Timeout::simpleTimeout(0);

    bool getSerialConfig(std::string file){
        tinyxml2::XMLDocument doc;
        tinyxml2::XMLError error = doc.LoadFile(file.c_str());
        
        if (error != tinyxml2::XML_SUCCESS) {
            std::cout << "Serial Config error: " << file.c_str() << std::endl;
            return false;
        }

        tinyxml2::XMLElement* root = doc.FirstChildElement("Serial_Config");
        if (!root) {
            std::cerr << "Missing <Serial_Config> root element" << std::endl;
            return false;
        }

        tinyxml2::XMLElement* portElem = root->FirstChildElement("port");
        if (portElem && portElem->GetText()) {
            port = portElem->GetText();
        } else {
            std::cerr << "Using default port: " << port << std::endl;
        }

        tinyxml2::XMLElement* baudElem = root->FirstChildElement("baud");
        if (baudElem && baudElem->GetText()) {
            baud = std::stoul(baudElem->GetText());
        } else {
            std::cerr << "Using default baud: " << baud << std::endl;
        }

        return true;
    }

    void enumerate_ports()
    {
        std::vector<serial::PortInfo> devices_found = serial::list_ports();
    
        std::vector<serial::PortInfo>::iterator iter = devices_found.begin();
    
        while( iter != devices_found.end() )
        {
            serial::PortInfo device = *iter++;
    
            printf( "(%s, %s, %s)\n", device.port.c_str(), device.description.c_str(),
         device.hardware_id.c_str() );
        }
    }
    
    void print_usage()
    {
        std::cerr << "Usage: test_serial {-e|<serial port address>} ";
        std::cerr << "<baudrate> [test string]" << std::endl;
    }


    
    CaveTalk_Error_t init(){
        std::string config_path = ament_index_cpp::get_package_share_directory("rover_comms") + "/src/Serial_Config.xml";
        bool getConfigSuccess = getSerialConfig(config_path);
        
        try{
            new_serial.setPort(port); // non-blocking essentially just 0s timeout
            new_serial.setBaudrate(baud);
            new_serial.setTimeout(timeout);
            new_serial.close();
            new_serial.open();

            if (new_serial.isOpen()) {
                std::cout << "Serial port opened successfully.\n";
                return CAVE_TALK_ERROR_NONE;
            } else {
                std::cerr << "Failed to open serial port.\n";
                return CAVE_TALK_ERROR_SOCKET_CLOSED;
            }

        }
        catch (const std::exception& e) {
            std::cerr << "Exception while initializing serial: " << e.what() << std::endl;
            return CAVE_TALK_ERROR_SOCKET_CLOSED;
        }
    }

    CaveTalk_Error_t deinit(){
        try{
            new_serial.close();
            return CAVE_TALK_ERROR_NONE;
        }
        catch (const std::exception& e) {
            std::cerr << "Exception while deinitializing serial: " << e.what() << std::endl;
            return CAVE_TALK_ERROR_SOCKET_CLOSED;
        }
    }

    CaveTalk_Error_t flush(){
        try{
            new_serial.flush();
            return CAVE_TALK_ERROR_NONE;
        }
        catch (const std::exception& e) {
            std::cerr << "Exception while flushing serial: " << e.what() << std::endl;
            return CAVE_TALK_ERROR_SOCKET_CLOSED;
        }
    }

    CaveTalk_Error_t send(const void *const data, const size_t size){
        CaveTalk_Error_t error = CAVE_TALK_ERROR_NONE;

        if(new_serial.isOpen()){
            try {
                size_t bytes_written = new_serial.write(static_cast<const uint8_t*>(data), size);
            }
            catch (const serial::IOException& e) {
                error = CAVE_TALK_ERROR_SOCKET_CLOSED;
            } 
        }
        else{
            error = CAVE_TALK_ERROR_SOCKET_CLOSED;
        }

    return error;
    }

    CaveTalk_Error_t receive(void *const data, const size_t size, size_t *const bytes_received){
        CaveTalk_Error_t error = CAVE_TALK_ERROR_NONE;

        if(new_serial.isOpen()){
            try {
                // read returns a std::string, so we need to convert
                std::string result = new_serial.read(size);
                *bytes_received = result.size();

                // Need to copy to the buffer
                memcpy(data, result.data(), *bytes_received);
            } 
            catch (const serial::IOException& e) {
                error = CAVE_TALK_ERROR_INCOMPLETE;
            } 
            catch (...) {
                error = CAVE_TALK_ERROR_INCOMPLETE;
            }
        }
        else{
            error = CAVE_TALK_ERROR_SOCKET_CLOSED;
        }

        return error;
    }

    std::string CaveTalk_ErrorToString(CaveTalk_Error_t error) {
        switch (error) {
            case CAVE_TALK_ERROR_NONE: return "CAVE_TALK_ERROR_NONE";
            case CAVE_TALK_ERROR_NULL: return "CAVE_TALK_ERROR_NULL";
            case CAVE_TALK_ERROR_SIZE: return "CAVE_TALK_ERROR_SIZE";
            case CAVE_TALK_ERROR_SOCKET_CLOSED: return "CAVE_TALK_ERROR_SOCKET_CLOSED";
            case CAVE_TALK_ERROR_INCOMPLETE: return "CAVE_TALK_ERROR_INCOMPLETE";
            case CAVE_TALK_ERROR_CRC: return "CAVE_TALK_ERROR_CRC";
            case CAVE_TALK_ERROR_VERSION: return "CAVE_TALK_ERROR_VERSION";
            case CAVE_TALK_ERROR_ID: return "CAVE_TALK_ERROR_ID";
            case CAVE_TALK_ERROR_PARSE: return "CAVE_TALK_ERROR_PARSE";
            default: return "UNKNOWN_ERROR";
        }
    }

} //end new_serial namespace
