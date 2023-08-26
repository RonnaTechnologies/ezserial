#include <ezserial/ezserial.hpp>

#include <string>

class SerialPort::SerialPortImpl
{

public:

    explicit SerialPortImpl(std::string_view port) : portName{port} 
    {
    }

    ~SerialPortImpl() = default;

    bool IsOpen() const noexcept { return isOpen; }

private:

    std::string portName;
    bool isOpen{false};

};

SerialPort::SerialPort(std::string_view portName) : impl{std::make_unique<SerialPortImpl>(portName)}
{

}

SerialPort::~SerialPort()
{

}

void SerialPort::Open() noexcept
{

}

void SerialPort::Close() noexcept
{

}

bool SerialPort::IsOpen() const noexcept
{
    return impl->IsOpen();
}

