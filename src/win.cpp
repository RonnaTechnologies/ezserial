#include <ezserial/ezserial.hpp>

#include <Windows.h>

#include <iostream>
#include <string>


class SerialPort::SerialPortImpl
{
public:
    explicit SerialPortImpl(std::string_view port) : portName{ R"(\\.\)" + std::string{ port } }
    {
    }

    ~SerialPortImpl()
    {
    }

    void Open() noexcept
    {
        hCom = CreateFile(portName.c_str(),             // port name
                          GENERIC_READ | GENERIC_WRITE, // Read/Write
                          0,                            // No Sharing
                          NULL,                         // No Security
                          OPEN_EXISTING,                // Open existing port only
                          0,                            // Non Overlapped I/O
                          NULL                          // Null for Comm Devices
        );

        if (hCom == INVALID_HANDLE_VALUE)
        {
            if (GetLastError() == ERROR_FILE_NOT_FOUND)
            {
                std::cout << "Serial port doesn't exist." << std::endl;
            }
            else
            {
                std::cout << "Unknown error..." << std::endl;
            }
            return;
        }

        // Configure serial port
        DCB serialParams = { 0 };
        serialParams.DCBlength = sizeof(serialParams);
        if (!GetCommState(hCom, &serialParams))
        {
            std::cout << "Failed to get serial port state." << std::endl;
            CloseHandle(hCom);
            return;
        }

        serialParams.fBinary = TRUE;
        serialParams.fDtrControl = DTR_CONTROL_ENABLE;
        serialParams.fDsrSensitivity = FALSE;
        serialParams.fTXContinueOnXoff = FALSE;
        serialParams.fOutX = FALSE;
        serialParams.fInX = FALSE;
        serialParams.fErrorChar = FALSE;
        serialParams.fNull = FALSE;
        serialParams.fRtsControl = RTS_CONTROL_ENABLE;
        serialParams.fAbortOnError = FALSE;
        serialParams.fOutxCtsFlow = FALSE;
        serialParams.fOutxDsrFlow = FALSE;
        serialParams.DCBlength = sizeof(serialParams);
        serialParams.BaudRate = 115200;
        serialParams.ByteSize = 8;
        serialParams.StopBits = ONESTOPBIT;
        serialParams.Parity = NOPARITY;
        serialParams.fBinary = TRUE;
        // serialParams.fDtrControl = DTR_CONTROL_ENABLE;
        serialParams.fRtsControl = RTS_CONTROL_ENABLE;

        if (!SetCommState(hCom, &serialParams))
        {
            std::cout << "Failed to set serial port state." << std::endl;
            CloseHandle(hCom);
            return;
        }

        // Set timeouts for non-blocking behavior
        COMMTIMEOUTS timeout = { 0 };
        timeout.ReadIntervalTimeout = 50;         // No interval timeout
        timeout.ReadTotalTimeoutConstant = 50;    // Return after 100ms if no data
        timeout.ReadTotalTimeoutMultiplier = 10;  // No multiplier
        timeout.WriteTotalTimeoutConstant = 50;   // Write timeout
        timeout.WriteTotalTimeoutMultiplier = 10; // Write multiplier

        if (!SetCommTimeouts(hCom, &timeout))
        {
            std::cout << "Failed to set serial port timeouts." << std::endl;
            CloseHandle(hCom);
            return;
        }
    }

    void Close() noexcept
    {
        if (hCom == nullptr)
        {
            return;
        }

        CloseHandle(hCom);
        hCom = nullptr;
    }

    bool IsOpen() const noexcept
    {
        return hCom != nullptr;
    }

    void SetBaudRate(std::size_t b)
    {
    }

    void Write(std::string_view data)
    {
        DWORD dwBytesWritten = 0;
        WriteFile(hCom, data.data(), data.size(), &dwBytesWritten, NULL);
    }

    std::string Read()
    {
        constexpr auto BUFFERLENGTH = 4095;
        char szBuff[BUFFERLENGTH + 1] = { 0 };
        DWORD dwBytesRead = 0;

        if (!ReadFile(hCom, szBuff, BUFFERLENGTH, &dwBytesRead, NULL))
        {
            DWORD error = GetLastError();
            if (error == ERROR_IO_PENDING)
            {
                // Overlapped I/O is not used here, so this should not happen
                return {};
            }
            else
            {
                // Other error (e.g., port closed)
                return {};
            }
        }

        if (dwBytesRead == 0)
        {
            // No data was read (timeout occurred)
            return {};
        }

        szBuff[dwBytesRead] = '\0'; // Null-terminate the string
        return std::string{ szBuff, dwBytesRead };
    }

private:
    HANDLE hCom = nullptr;
    std::string portName;
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
