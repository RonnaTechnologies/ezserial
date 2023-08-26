
include(CPM)
CPMAddPackage("gh:catchorg/Catch2@2.13.9")

add_executable(ezserial_test test.cpp)
target_link_libraries(ezserial_test PRIVATE ezserial Catch2::Catch2)

target_compile_features(ezserial_test PRIVATE cxx_std_23)
set_target_properties(ezserial_test PROPERTIES CXX_EXTENSIONS OFF)

add_test(NAME test_ezserial COMMAND ezserial_test)