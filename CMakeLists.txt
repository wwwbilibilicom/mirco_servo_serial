cmake_minimum_required(VERSION 3.10)

# Project name and version
project(mirco_servo_serial 
    VERSION 1.0.0
    DESCRIPTION "Servo motor control via serial communication"
    LANGUAGES CXX
)

# Set C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Set default build type to Release if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "-O2")

# Include directories
include_directories(${PROJECT_SOURCE_DIR}/include)

# Source files for the library
set(LIB_SOURCES
    src/servo_controller.cpp
)

# Header files
set(LIB_HEADERS
    include/servo_controller.h
)

# Create static library
add_library(servo_controller STATIC ${LIB_SOURCES})
target_include_directories(servo_controller PUBLIC 
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Create shared library (optional)
add_library(servo_controller_shared SHARED ${LIB_SOURCES})
target_include_directories(servo_controller_shared PUBLIC 
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
set_target_properties(servo_controller_shared PROPERTIES 
    OUTPUT_NAME servo_controller
    VERSION ${PROJECT_VERSION}
    SOVERSION 1
)

# Example executable
add_executable(servo_example example/mirco_servo_serial.cpp)
target_link_libraries(servo_example PRIVATE servo_controller)

# Installation rules
install(TARGETS servo_example servo_controller servo_controller_shared
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

install(FILES ${LIB_HEADERS} DESTINATION include)

# Print build information
message(STATUS "")
message(STATUS "========================================")
message(STATUS "  ${PROJECT_NAME} v${PROJECT_VERSION}")
message(STATUS "  Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "  C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "========================================")
message(STATUS "")
