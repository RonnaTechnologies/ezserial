#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>

#include <ezserial/ezserial.hpp>

TEST_CASE( "Quick check", "[main]" ) 
{
    SerialPort sp{"test"};
    REQUIRE( !sp.IsOpen() );
}
