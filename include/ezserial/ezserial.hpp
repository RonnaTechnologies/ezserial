#pragma once

#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <string_view>

class SerialPort
{
public:
    explicit SerialPort(std::string_view portName);
    explicit SerialPort(int nativeHandle);
    virtual ~SerialPort();

    SerialPort(const SerialPort&) = delete;
    SerialPort(SerialPort&&) = delete;
    auto operator=(const SerialPort&) = delete;
    auto operator=(SerialPort&&) = delete;

    void Open() noexcept;
    void Close() noexcept;

    bool Poll(const std::optional<std::chrono::milliseconds>& timeout) noexcept;

    void Write(std::string_view data);
    std::string Read();

    void SetBaudRate(std::size_t baudRate) noexcept;

    [[nodiscard]] bool IsOpen() const noexcept;


private:
    class SerialPortImpl;
    std::unique_ptr<SerialPortImpl> impl;
};