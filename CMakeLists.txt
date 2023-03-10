cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

include(${CMAKE_SOURCE_DIR}/RP2040-HAT-C/pico_sdk_import.cmake)
include(${CMAKE_SOURCE_DIR}/RP2040-HAT-C/rp2040_hat_c_sdk_version.cmake)

project(demo C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

# -----------------------------------------------------------------------------

# Set ethernet chip
set(WIZNET_CHIP W5100S)

if(${WIZNET_CHIP} STREQUAL W5100S)
  add_definitions(-D_WIZCHIP_=W5100S)
elseif(${WIZNET_CHIP} STREQUAL W5500)
  add_definitions(-D_WIZCHIP_=W5500)
else()
  message(FATAL_ERROR WIZNET_CHIP is wrong = '${WIZNET_CHIP}')
endif()

message(STATUS WIZNET_CHIP = '${WIZNET_CHIP}')

# Set the project root directory if it's not already defined, as may happen if
# the tests folder is included directly by a parent project, without including
# the top level CMakeLists.txt.
if(NOT DEFINED WIZNET_DIR)
    set(WIZNET_DIR ${CMAKE_SOURCE_DIR}/RP2040-HAT-C/libraries/ioLibrary_Driver)
    message(STATUS "WIZNET_DIR = ${WIZNET_DIR}")
endif()

if(NOT DEFINED MBEDTLS_DIR)
    set(MBEDTLS_DIR ${CMAKE_SOURCE_DIR}/RP2040-HAT-C/libraries/mbedtls)
    message(STATUS "MBEDTLS_DIR = ${MBEDTLS_DIR}")
endif()

if(NOT DEFINED PORT_DIR)
    set(PORT_DIR ${CMAKE_SOURCE_DIR}/RP2040-HAT-C/port)
    message(STATUS "PORT_DIR = ${PORT_DIR}")
endif()

# Turn off mbedtls test mode 
set(ENABLE_PROGRAMS OFF CACHE BOOL "Build mbedtls programs")
set(ENABLE_TESTING OFF CACHE BOOL "Build mbedtls testing")
add_definitions(-DMBEDTLS_CONFIG_FILE="${PORT_DIR}/mbedtls/inc/ssl_config.h")
add_definitions(-DSET_TRUSTED_CERT_IN_SAMPLES)

# -----------------------------------------------------------------------------

add_executable(demo
                demo.c
                pico-eeprom.h
                pico-eeprom.c
                callbacks-link.c
                callbacks-vscp-protocol.c
                ${CMAKE_SOURCE_DIR}/../../VSCP/vscp-firmware/common/vscp-fifo.h 
                ${CMAKE_SOURCE_DIR}/../../VSCP/vscp-firmware/common/vscp-fifo.c
                ${CMAKE_SOURCE_DIR}/../../VSCP/vscp-firmware/common/vscp-firmware-helper.h 
                ${CMAKE_SOURCE_DIR}/../../VSCP/vscp-firmware/common/vscp-firmware-helper.c
                ${CMAKE_SOURCE_DIR}/../../VSCP/vscp-firmware/common/vscp-link-protocol.c
                ${CMAKE_SOURCE_DIR}/../../VSCP/vscp-firmware/common/vscp-link-protocol.h
                ${CMAKE_SOURCE_DIR}/../../VSCP/vscp-firmware/common/vscp-firmware-level2.c
                ${CMAKE_SOURCE_DIR}/../../VSCP/vscp-firmware/common/vscp-firmware-level2.h
)

# Add libraries in subdirectories
add_subdirectory(${CMAKE_SOURCE_DIR}/RP2040-HAT-C/libraries)
add_subdirectory(${MBEDTLS_DIR})
add_subdirectory(${PORT_DIR})

include_directories( PUBLIC . 
                      ${CMAKE_SOURCE_DIR}/../../VSCP/vscp-firmware/common
                    )

target_link_libraries(demo  
                        pico_stdlib
                        pico_multicore
                        hardware_spi
                        hardware_dma
                        hardware_adc
                        hardware_flash
                        hardware_gpio
                        hardware_rtc
                        ETHERNET_FILES
                        IOLIBRARY_FILES
                        LOOPBACK_FILES
                        SNTP_FILES
                        TIMER_FILES
                        )

pico_enable_stdio_usb(demo 1)
pico_enable_stdio_uart(demo 1)

pico_add_extra_outputs(demo)

