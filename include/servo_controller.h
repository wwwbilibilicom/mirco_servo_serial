/**
 * @file servo_controller.h
 * @brief ServoController class for serial communication with servo motors
 * @date 2026-01-31
 * 
 * This class provides an object-oriented interface for controlling servo motors
 * via UART serial communication without external dependencies.
 */

#ifndef SERVO_CONTROLLER_H
#define SERVO_CONTROLLER_H

#include <cstdint>
#include <string>

/**
 * @class ServoController
 * @brief Manages serial communication with servo motors
 * 
 * Provides methods to open serial ports, configure communication parameters,
 * and send angle commands to servo motors.
 */
class ServoController {
public:
    /**
     * @brief Default constructor
     */
    ServoController();
    
    /**
     * @brief Parameterized constructor with auto-initialization
     * @param port Serial port device path (e.g., "/dev/ttyUSB0")
     * @param baudrate Communication baud rate (default: 921600)
     */
    ServoController(const std::string& port, unsigned int baudrate = 921600);
    
    /**
     * @brief Destructor - automatically closes serial port
     */
    ~ServoController();
    
    /**
     * @brief Initialize and open serial port
     * @param port Serial port device path
     * @param baudrate Communication baud rate (default: 921600)
     * @return true if successful, false otherwise
     */
    bool init(const std::string& port, unsigned int baudrate = 921600);
    
    /**
     * @brief Send angle command to specific servo
     * @param servo_id Servo ID (0-255)
     * @param angle Target angle in degrees (0.0 - 180.0)
     * @return true if command sent successfully, false otherwise
     */
    bool setServoAngle(uint8_t servo_id, float angle);
    
    /**
     * @brief Check if serial port is open
     * @return true if port is open, false otherwise
     */
    bool isOpen() const;
    
    /**
     * @brief Close serial port
     */
    void close();
    
    /**
     * @brief Get last error message
     * @return Error message string
     */
    std::string getLastError() const;
    
private:
    /**
     * @brief Configure serial port parameters
     * @param baudrate Desired baud rate
     * @return true if configuration successful, false otherwise
     */
    bool configurePort(unsigned int baudrate);
    
    /**
     * @brief Validate angle range
     * @param angle Angle to validate
     * @return true if angle is within valid range [0.0, 180.0]
     */
    bool validateAngle(float angle) const;
    
    /**
     * @brief Set error message
     * @param error Error message to store
     */
    void setError(const std::string& error);
    
    int m_fd;                    ///< File descriptor for serial port
    std::string m_port;          ///< Serial port device path
    std::string m_lastError;     ///< Last error message
    bool m_isOpen;               ///< Port open status flag
};

#endif // SERVO_CONTROLLER_H
