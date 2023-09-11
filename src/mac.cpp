#include <ezserial/ezserial.hpp>

#include <algorithm>
#include <cstdio>      // Standard input / output functions
#include <cstdlib>
#include <cstring>     // String function definitions
#include <filesystem>
#include <string>
#include <unordered_map>

#include <unistd.h>     // UNIX standard function definitions
#include <fcntl.h>      // File control definitions
#include <termios.h>    // POSIX terminal control definitions

class SerialPort::SerialPortImpl
{

public:

    explicit SerialPortImpl(std::string_view port) : portName{port} 
    {
    }

    ~SerialPortImpl()
    {

    }

    void Open() noexcept
    {
        handle = ::open(portName.c_str(), O_RDWR);
        termios tty;


        if(tcgetattr(handle, &tty) != 0)
        {
            return;
        }

        const auto baudRateIt = baudRates.find(baudRate);
        if(baudRateIt == baudRates.cend())
        {
            return;
        }

        const auto rate = baudRateIt->second;

            // Set baud rate
        cfsetospeed(&tty, rate);
        cfsetispeed(&tty, rate);

        tty.c_cflag &= ~PARENB;            // Make 8n1
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;

        tty.c_cflag &=  ~CRTSCTS;           // no flow control
        tty.c_cc[VMIN] = 0;                 // read blocks
        tty.c_cc[VTIME] = 10;               // 1 second read timeout
        tty.c_cflag |= CREAD | CLOCAL;     // turn on READ & ignore ctrl lines

        // Raw mode
        cfmakeraw(&tty);

        // Flush Port, then applies attributes
        tcflush(this->handle, TCIFLUSH);

        if(tcsetattr(this->handle, TCSANOW, &tty) != 0)
        {
            return;
        }
    }

    void Close() noexcept
    {
        if(isOpen && handle != -1)
        {
            ::close(handle);
            isOpen = false;
            handle = -1;
        }
    }

    void Write(std::string_view data)
    {
        ::write(handle, data.data(), data.size());
    }

    std::string Read()
    {
        char smallBuff[256] = {0};
        
        ::read(handle, &smallBuff, sizeof(smallBuff));

        return std::string{&smallBuff[0]};
    }

    void SetBaudRate(std::size_t baudRate) noexcept
    {
        this->baudRate = baudRate;
    }

    bool IsOpen() const noexcept { return isOpen; }

private:

    std::string portName;
    bool isOpen{false};
    std::size_t baudRate;
    int handle{};

    inline static std::unordered_map<std::size_t, speed_t> baudRates
    {
        {0UL, B0},
        {50UL, B50},
        {75UL, B75},
        {110UL, B110},
        {134UL, B134},
        {150UL, B150},
        {200UL, B200},
        {300UL, B300},
        {600UL, B600},
        {1200UL, B1200},
        {1800UL, B1800},
        {2400UL, B2400},
        {4800UL, B4800},
        {9600UL, B9600},
        {19200UL, B19200},
        {38400UL, B38400},
        {57600UL, B57600},
        {115200UL, B115200},
        {230400UL, B230400},
    };

};

SerialPort::SerialPort(std::string_view portName)
: impl{std::make_unique<SerialPortImpl>(portName)}
{

}

SerialPort::~SerialPort()
{

}

void SerialPort::Open() noexcept
{
    impl->Open();
}

void SerialPort::Close() noexcept
{
    impl->Close();
}

void SerialPort::Write(std::string_view data)
{
    impl->Write(data);
}

std::string SerialPort::Read()
{
    return impl->Read();
}

void SerialPort::SetBaudRate(std::size_t baudRate) noexcept
{
    impl->SetBaudRate(baudRate);
}

bool SerialPort::IsOpen() const noexcept
{
    return impl->IsOpen();
}
