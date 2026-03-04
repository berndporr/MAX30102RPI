cmake_minimum_required(VERSION 3.20)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
	set(CMAKE_BUILD_TYPE "RelWithDebInfo")
endif()

project(MAX30102_RaspberryPi_Library VERSION 1.0.0 LANGUAGES CXX)
include(GNUInstallDirs)

add_compile_options(-Wall -Wconversion -Wextra -pedantic)

set(LIBSRC max30102rpi.cpp)
set(LIBINCLUDE max30102rpi.h)

add_library(max30102rpi
  SHARED
  ${LIBSRC}
  )

set_target_properties(max30102rpi PROPERTIES
  SOVERSION 1
  PUBLIC_HEADER "${LIBINCLUDE}")

target_link_libraries(max30102rpi gpiod)

install(TARGETS max30102rpi EXPORT max30102rpi-targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})


add_library(max30102rpi_static
  STATIC
  ${LIBSRC}
  )

set_target_properties(max30102rpi_static PROPERTIES
  PUBLIC_HEADER "${LIBINCLUDE}")

target_link_libraries(max30102rpi_static gpiod)

install(TARGETS max30102rpi_static EXPORT max30102rpi_static-targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
