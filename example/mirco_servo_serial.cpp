/**
 * @file mirco_servo_serial.cpp
 * @brief Main application for servo control via serial communication
 * @date 2026-01-31
 * 
 * Demonstrates the usage of ServoController class for controlling
 * servo motors through UART serial communication.
 */

#include "servo_controller.h"
#include <unistd.h>
#include <iostream>

int main() {
    // Create servo controller instance
    ServoController controller;
    
    // Initialize serial port
    std::cout << "=== Servo Controller Demo ===" << std::endl;
    std::cout << "Initializing serial port..." << std::endl;
    
    if (!controller.init("/dev/ttyUSB0", 921600)) {
        std::cerr << "Failed to initialize: " << controller.getLastError() << std::endl;
        return -1;
    }
    
    std::cout << "\n--- Example 1: Basic servo control ---" << std::endl;
    
    // Set servo 0 to 90 degrees
    if (controller.setServoAngle(0, 90.0f)) {
        std::cout << "Command sent successfully" << std::endl;
    } else {
        std::cerr << "Failed: " << controller.getLastError() << std::endl;
    }
    sleep(1);
    
    // Set servo 0 to 45 degrees
    if (controller.setServoAngle(0, 45.0f)) {
        std::cout << "Command sent successfully" << std::endl;
    } else {
        std::cerr << "Failed: " << controller.getLastError() << std::endl;
    }
    sleep(1);
    
    // Set servo 0 to 135 degrees
    if (controller.setServoAngle(0, 135.0f)) {
        std::cout << "Command sent successfully" << std::endl;
    } else {
        std::cerr << "Failed: " << controller.getLastError() << std::endl;
    }
    sleep(1);
    
    std::cout << "\n--- Example 2: Smooth sweep ---" << std::endl;
    
    // Sweep from 0 to 180 degrees
    for (float angle = 0.0f; angle <= 180.0f; angle += 15.0f) {
        if (!controller.setServoAngle(0, angle)) {
            std::cerr << "Failed: " << controller.getLastError() << std::endl;
        }
        usleep(300000);  // 300ms delay
    }
    
    std::cout << "\n--- Example 3: Multiple servos ---" << std::endl;
    
    // Control multiple servos
    for (uint8_t servo_id = 0; servo_id < 3; servo_id++) {
        std::cout << "Setting servo " << (int)servo_id << " to 90 degrees" << std::endl;
        if (!controller.setServoAngle(servo_id, 90.0f)) {
            std::cerr << "Failed: " << controller.getLastError() << std::endl;
        }
        usleep(500000);  // 500ms delay
    }
    
    std::cout << "\n--- Demo completed ---" << std::endl;
    
    // Controller will automatically close in destructor
    controller.close();
    
    return 0;
}