#include <ezserial/ezserial.hpp>

#include <chrono>
#include <iostream>
#include <span>
#include <thread>

using namespace std::chrono_literals;
using namespace std::this_thread;

int main(int argc, const char** const argv)
{
    const auto args = std::span(argv, argc);

    if (args.size() != 2)
    {
        return EXIT_FAILURE;
    }

    const auto portName = args.back();

    SerialPort sp{ portName };
    sp.SetBaudRate(115'200);
    sp.Open();
    while (!sp.Poll(10ms))
        ;
    std::cout << sp.Read() << std::endl;

    sp.Close();

    return EXIT_SUCCESS;
}