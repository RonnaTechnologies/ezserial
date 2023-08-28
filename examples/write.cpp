#include <ezserial/ezserial.hpp>

#include <iostream>
#include <span>

int main(int argc, const char** const argv)
{
    const auto args = std::span(argv, argc);

    if(args.size() !=  3)
    {
        return EXIT_FAILURE;
    }

    const auto portName = args[1];
    const auto data = args.back();

    SerialPort sp{portName};
    sp.SetBaudRate(115'200);
    sp.Open();
    sp.Write(data);
    sp.Close();

    return EXIT_SUCCESS;
}