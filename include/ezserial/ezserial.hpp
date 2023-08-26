#pragma once

#include <memory>
#include <string>
#include <string_view>

class SerialPort
{
public:

    explicit SerialPort(std::string_view portName);
    virtual ~SerialPort();

    void Open() noexcept;
    void Close() noexcept;

    std::string Read();

    void SetBaudRate(std::size_t baudRate) noexcept;

    bool IsOpen() const noexcept;


private:

    class SerialPortImpl;
    std::unique_ptr<SerialPortImpl> impl;
};