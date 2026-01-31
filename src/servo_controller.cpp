/**
 * @file servo_controller.cpp
 * @brief Implementation of ServoController class
 * @date 2026-01-31
 */

#include "servo_controller.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <sstream>
#include <iomanip>

// Constructor
ServoController::ServoController() 
    : m_fd(-1)
    , m_port("")
    , m_lastError("")
    , m_isOpen(false) {
}

// Parameterized constructor
ServoController::ServoController(const std::string& port, unsigned int baudrate)
    : m_fd(-1)
    , m_port("")
    , m_lastError("")
    , m_isOpen(false) {
    init(port, baudrate);
}

// Destructor
ServoController::~ServoController() {
    close();
}

// Initialize and open serial port
bool ServoController::init(const std::string& port, unsigned int baudrate) {
    // Close existing connection if any
    if (m_isOpen) {
        close();
    }
    
    m_port = port;
    
    // Open serial port
    m_fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (m_fd < 0) {
        std::ostringstream oss;
        oss << "Failed to open port " << port << ": " << strerror(errno);
        setError(oss.str());
        return false;
    }
    
    // Configure port parameters
    if (!configurePort(baudrate)) {
        ::close(m_fd);
        m_fd = -1;
        return false;
    }
    
    m_isOpen = true;
    m_lastError = "";
    
    printf("Serial port opened successfully: %s @ %d bps\n", port.c_str(), baudrate);
    return true;
}

// Configure serial port parameters
bool ServoController::configurePort(unsigned int baudrate) {
    struct termios options;
    
    // Get current port settings
    if (tcgetattr(m_fd, &options) != 0) {
        setError("Failed to get terminal attributes: " + std::string(strerror(errno)));
        return false;
    }
    
    // Determine baud rate constant
    speed_t baud_const;
    switch (baudrate) {
        case 9600:    baud_const = B9600;    break;
        case 19200:   baud_const = B19200;   break;
        case 38400:   baud_const = B38400;   break;
        case 57600:   baud_const = B57600;   break;
        case 115200:  baud_const = B115200;  break;
        case 230400:  baud_const = B230400;  break;
        case 460800:  baud_const = B460800;  break;
        case 921600:  baud_const = B921600;  break;
        default:
            setError("Unsupported baud rate");
            return false;
    }
    
    // Set baud rate
    cfsetispeed(&options, baud_const);
    cfsetospeed(&options, baud_const);
    
    // Configure port settings
    options.c_cflag |= (CLOCAL | CREAD);  // Enable receiver, ignore modem control lines
    options.c_cflag &= ~CSTOPB;           // 1 stop bit
    options.c_cflag &= ~CSIZE;            // Clear data size bits
    options.c_cflag |= CS8;               // 8 data bits
    options.c_cflag &= ~PARENB;           // No parity
    
    // Raw input mode
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    
    // Raw output mode
    options.c_oflag &= ~OPOST;
    
    // Set timeout (non-blocking)
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 0;
    
    // Apply settings
    if (tcsetattr(m_fd, TCSANOW, &options) != 0) {
        setError("Failed to set terminal attributes: " + std::string(strerror(errno)));
        return false;
    }
    
    // Flush input/output buffers
    tcflush(m_fd, TCIOFLUSH);
    
    return true;
}

// Send angle command to servo
bool ServoController::setServoAngle(uint8_t servo_id, float angle) {
    // Check if port is open
    if (!m_isOpen || m_fd < 0) {
        setError("Serial port is not open");
        return false;
    }
    
    // Validate angle range
    if (!validateAngle(angle)) {
        std::ostringstream oss;
        oss << "Angle out of range [0.0, 180.0]: " << angle;
        setError(oss.str());
        return false;
    }
    
    // Build command string: "servo_id:angle\r\n"
    char cmd[64];
    int len = snprintf(cmd, sizeof(cmd), "%d:%.1f\r\n", servo_id, angle);
    
    if (len < 0 || len >= (int)sizeof(cmd)) {
        setError("Failed to format command");
        return false;
    }
    
    // Send command
    ssize_t written = write(m_fd, cmd, len);
    if (written != len) {
        std::ostringstream oss;
        oss << "Failed to write complete command: wrote " << written 
            << " of " << len << " bytes";
        setError(oss.str());
        return false;
    }
    
    printf("Sent: %s", cmd);
    m_lastError = "";
    return true;
}

// Check if port is open
bool ServoController::isOpen() const {
    return m_isOpen && m_fd >= 0;
}

// Close serial port
void ServoController::close() {
    if (m_fd >= 0) {
        ::close(m_fd);
        printf("Serial port closed: %s\n", m_port.c_str());
        m_fd = -1;
    }
    m_isOpen = false;
}

// Get last error message
std::string ServoController::getLastError() const {
    return m_lastError;
}

// Validate angle range
bool ServoController::validateAngle(float angle) const {
    return (angle >= 0.0f && angle <= 180.0f);
}

// Set error message
void ServoController::setError(const std::string& error) {
    m_lastError = error;
    fprintf(stderr, "Error: %s\n", error.c_str());
}
