#include <array>
#include <ezserial/ezserial.hpp>

#include <array>
#include <cstdio> // Standard input / output functions
#include <cstdlib>
#include <cstring> // String function definitions
#include <string>
#include <unordered_map>

#include <cerrno>  // Error number definitions
#include <fcntl.h> // File control definitions
#include <sys/epoll.h>
#include <termios.h> // POSIX terminal control definitions
#include <unistd.h>  // UNIX standard function definitions

class SerialPort::SerialPortImpl
{

public:
    explicit SerialPortImpl(std::string_view port) : portName{ port }
    {
    }

    ~SerialPortImpl()
    {
    }

    void Open() noexcept
    {
        handle = ::open(portName.c_str(), O_RDWR | O_NOCTTY);
        termios tty;

        if (tcgetattr(handle, &tty) != 0)
        {
            return;
        }

        const auto baudRateIt = baudRates.find(baudRate);
        if (baudRateIt == baudRates.cend())
        {
            return;
        }

        const auto rate = baudRateIt->second;

        // Set baud rate
        cfsetospeed(&tty, rate);
        cfsetispeed(&tty, rate);

        tty.c_cflag &= ~PARENB; // Make 8n1 (Disable parity)
        tty.c_cflag &= ~CSTOPB; // Use a single stop bit
        tty.c_cflag &= ~CSIZE;  // Clear all the size bits
        tty.c_cflag |= CS8;     // 8 bits per byte

        tty.c_cflag &= ~CRTSCTS;       // Disable flow control
        tty.c_cc[VMIN] = 0;            // Read blocks
        tty.c_cc[VTIME] = 1;           // 1 second read timeout
        tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines

        // Raw mode
        cfmakeraw(&tty);

        // Flush Port, then applies attributes
        tcflush(this->handle, TCIFLUSH);

        if (tcsetattr(this->handle, TCSANOW, &tty) != 0)
        {
            return;
        }

        isOpen = true;
    }

    void Close() noexcept
    {
        if (isOpen && handle != -1)
        {
            ::close(handle);
            isOpen = false;
            handle = -1;
        }
    }

    bool Poll(const std::optional<std::chrono::milliseconds>& timeout) noexcept
    {
        std::array<::epoll_event, 1U> e{};
        if (epoll_handle == -1)
        {
            epoll_handle = ::epoll_create1(0);
            e.front().events |= ::EPOLLRDNORM;

            ::epoll_ctl(epoll_handle, EPOLL_CTL_ADD, handle, e.data());
        }

        e.front().events = 0;
        const auto timeout_ms = timeout.has_value() ? timeout->count() : -1;
        ::epoll_wait(epoll_handle, e.data(), e.size(), timeout_ms);

        return e.front().events | ::EPOLLRDNORM;
    }

    void Write(std::string_view data)
    {
        ::write(handle, data.data(), data.size());
    }

    std::string Read()
    {
        char smallBuff[256] = { 0 };
        ::read(handle, &smallBuff, sizeof(smallBuff));

        return std::string{ &smallBuff[0] };
    }

    void SetBaudRate(std::size_t baudRate) noexcept
    {
        this->baudRate = baudRate;
    }

    bool IsOpen() const noexcept
    {
        return isOpen;
    }

private:
    std::string portName;
    bool isOpen{ false };
    std::size_t baudRate;
    int handle{};
    int epoll_handle{ -1 };

    inline static std::unordered_map<std::size_t, speed_t> baudRates{
        { 0UL, B0 },
        { 50UL, B50 },
        { 75UL, B75 },
        { 110UL, B110 },
        { 134UL, B134 },
        { 150UL, B150 },
        { 200UL, B200 },
        { 300UL, B300 },
        { 600UL, B600 },
        { 1200UL, B1200 },
        { 1800UL, B1800 },
        { 2400UL, B2400 },
        { 4800UL, B4800 },
        { 9600UL, B9600 },
        { 19200UL, B19200 },
        { 38400UL, B38400 },
        { 57600UL, B57600 },
        { 115200UL, B115200 },
        { 230400UL, B230400 },
        { 460800UL, B460800 },
        { 500000UL, B500000 },
        { 576000UL, B576000 },
        { 921600UL, B921600 },
        { 1000000UL, B1000000 },
        { 1152000UL, B1152000 },
        { 1500000UL, B1500000 },
        { 2000000UL, B2000000 },
        { 2500000UL, B2500000 },
        { 3000000UL, B3000000 },
        { 3500000UL, B3500000 },
        { 4000000UL, B4000000 },
    };
};

SerialPort::SerialPort(std::string_view portName) : impl{ std::make_unique<SerialPortImpl>(portName) }
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

bool SerialPort::Poll(const std::optional<std::chrono::milliseconds>& timeout) noexcept
{
    return impl->Poll(timeout);
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
