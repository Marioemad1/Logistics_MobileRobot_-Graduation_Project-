#ifndef SHARED_SERIAL_PORT_HPP
#define SHARED_SERIAL_PORT_HPP

#include <libserial/SerialPort.h>
#include <memory>
#include <mutex>

namespace hw_interface
{

class SharedSerialPort {
public:
    // This is the magic Singleton function. It always returns the exact same object.
    static SharedSerialPort& getInstance() {
        static SharedSerialPort instance; 
        return instance;
    }

    // The shared serial port variable
    std::shared_ptr<LibSerial::SerialPort> port;
    
    // The lock to prevent both plugins from talking at the same time
    std::mutex serial_mutex; 

private:
    // Private constructor so no one else can accidentally create a second one!
    SharedSerialPort() {
        port = std::make_shared<LibSerial::SerialPort>();
    }

    // Delete copy constructors to enforce the Singleton rule
    SharedSerialPort(SharedSerialPort const&) = delete;
    void operator=(SharedSerialPort const&) = delete;
};

} // namespace hw_interface

#endif