cmake_minimum_required(VERSION 3.15)
project(EZSerial VERSION 0.1.0 DESCRIPTION "Serial port C++ library" LANGUAGES CXX)

include(CTest)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

option(BUILD_SHARED_LIBS "Build using shared libraries" OFF)

if(PROJECT_SOURCE_DIR STREQUAL PROJECT_BINARY_DIR)
  message(
    FATAL_ERROR
      "In-source builds not allowed. Please make a new directory (called a build directory) and run CMake from there."
  )
endif()

message(STATUS "Operating System: ${CMAKE_SYSTEM_NAME}")

if(${CMAKE_SYSTEM_NAME} STREQUAL "Windows")
    list(APPEND EZSerial_src "src/win.cpp")
endif()

if(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
    list(APPEND EZSerial_src "src/unix.cpp")
endif()

if(${CMAKE_SYSTEM_NAME} STREQUAL "Darwin")
    list(APPEND EZSerial_src "src/mac.cpp")
endif()

add_library(ezserial ${EZSerial_src})

target_include_directories(ezserial
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_compile_features(ezserial PRIVATE cxx_std_23)
set_target_properties(ezserial PROPERTIES CXX_EXTENSIONS OFF)

source_group(TREE "${EZSerial_SOURCE_DIR}/include" PREFIX "Header Files" FILES ${HEADER_LIST})

if((CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME OR MODERN_CMAKE_BUILD_TESTING)
   AND BUILD_TESTING)
  add_subdirectory(tests)
endif()
