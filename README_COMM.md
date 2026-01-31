# Servo Communication Driver Documentation

## Overview

This document describes the serial communication protocol and implementation for servo control. It enables real-time servo angle control through UART serial communication using DMA interrupt mode to receive commands from a host computer.

## Hardware Configuration

| Parameter | Value |
|-----------|-------|
| Serial Port | USART1 |
| Baud Rate | 921600 bps |
| Data Bits | 8 bit |
| Stop Bits | 1 bit |
| Parity | None |
| DMA RX | DMA1_Channel5 |
| DMA TX | DMA1_Channel4 |

## Communication Protocol

### Command Format

```
<servo_id>:<angle>\r\n
```

### Parameter Description

| Parameter | Description | Range | Example |
|-----------|-------------|-------|---------|
| `servo_id` | Servo ID (0-based index) | 0-255 | 0 |
| `angle` | Target angle (float) | 0.0 - 180.0 | 90.5 |
| `\r\n` | Line terminator | - | CR+LF |

### Angle to PWM Duty Cycle Mapping

- **0°** → PWM Duty Cycle **2.5%**
- **180°** → PWM Duty Cycle **12.5%**
- PWM Frequency: **50Hz**
- Linear mapping: `duty = 2.5 + (angle / 180.0) * 10.0`

### Command Examples

```
0:0.0\r\n        # Set servo 0 to 0 degrees (minimum position)
0:90.5\r\n       # Set servo 0 to 90.5 degrees (middle position)
0:180.0\r\n      # Set servo 0 to 180 degrees (maximum position)
0:45.0\r\n       # Set servo 0 to 45 degrees
1:120.5\r\n      # Set servo 1 to 120.5 degrees
```

### Response Format

| Response Type | Format | Description |
|---------------|--------|-------------|
| Success | `Servo X set to Y.Y degrees\r\n` | Servo setting successful |
| Error | `Error setting servo X\r\n` | Servo setting failed |

## Driver API Reference

### Initialization Function

```c
int drv_comm_init(CommDrv_t *drv, UART_HandleTypeDef *huart);
```

**Description**: Initialize the communication driver
- **Parameters**:
  - `drv`: Pointer to communication driver structure
  - `huart`: Pointer to UART handle
- **Return Value**: 0 for success, -1 for failure

### Start Reception Function

```c
int drv_comm_start_rx(CommDrv_t *drv);
```

**Description**: Start DMA UART reception
- **Parameters**: `drv` - Pointer to communication driver structure
- **Return Value**: 0 for success, -1 for failure

### Data Processing Function

```c
void drv_comm_on_data_received(CommDrv_t *drv, uint8_t *data, uint16_t len);
```

**Description**: Process received data (called from UART idle interrupt)
- **Parameters**:
  - `drv`: Pointer to communication driver structure
  - `data`: Pointer to received data buffer
  - `len`: Data length

### Apply Command Function

```c
int drv_comm_apply_cmd(CommDrv_t *drv, void *servo_ptr, uint8_t servo_id);
```

**Description**: Parse and apply servo command
- **Parameters**:
  - `drv`: Pointer to communication driver structure
  - `servo_ptr`: Pointer to servo structure
  - `servo_id`: Servo ID
- **Return Value**:
  - 0 - Command executed successfully
  - 1 - No command ready
  - -1 - Parse error

## Firmware Integration Example

### main.c Initialization Code

```c
/* In the initialization section */
#include "drv_comm.h"

CommDrv_t comm_driver;  /* Global communication driver structure */

int main(void)
{
    /* ... other initialization code ... */
    
    /* Initialize communication driver */
    drv_comm_init(&comm_driver, &huart1);
    drv_comm_start_rx(&comm_driver);
    
    printf("Servo communication ready\r\n");
    
    while (1) {
        /* Process commands in main loop */
        drv_comm_apply_cmd(&comm_driver, &servo1, 0);
    }
}
```

### Interrupt Handler Code (stm32f1xx_it.c)

```c
extern CommDrv_t comm_driver;

void USART1_IRQHandler(void)
{
    /* Detect UART Idle Line condition */
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE)) {
        __HAL_UART_CLEAR_IDLEFLAG(&huart1);
        
        /* Get remaining data count from DMA */
        uint32_t dma_cnt = __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
        uint16_t data_len = COMM_RX_BUF_SIZE - dma_cnt;
        
        if (data_len > 0) {
            /* Process received data */
            drv_comm_on_data_received(&comm_driver, comm_driver.rx_buf, data_len);
        }
        
        /* Restart DMA */
        HAL_DMA_Abort(&hdma_usart1_rx);
        HAL_UART_Receive_DMA(&huart1, comm_driver.rx_buf, COMM_RX_BUF_SIZE);
    }
    
    HAL_UART_IRQHandler(&huart1);
}
```

## Host Computer C++ Implementation

### Complete Implementation with ROS Serial Library

```cpp
#include <serial/serial.h>  // ROS serial library
#include <iostream>
#include <sstream>
#include <iomanip>

class ServoController {
private:
    serial::Serial ser;
    
public:
    /**
     * @brief Initialize serial port connection
     * @param port Serial port name (e.g., "/dev/ttyUSB0" or "COM3")
     * @param baudrate Baud rate (default 921600)
     * @return true if connection successful
     */
    bool init(const std::string &port, uint32_t baudrate = 921600) {
        try {
            ser.setPort(port);
            ser.setBaudrate(baudrate);
            serial::Timeout to = serial::Timeout::simpleTimeout(1000);
            ser.setTimeout(to);
            ser.open();
            
            if (ser.isOpen()) {
                std::cout << "Serial port opened successfully: " << port << std::endl;
                return true;
            }
        } catch (serial::SerialException &e) {
            std::cerr << "Error opening serial port: " << e.what() << std::endl;
        }
        return false;
    }
    
    /**
     * @brief Set servo angle
     * @param servo_id Servo ID
     * @param angle Target angle (0.0 - 180.0)
     * @return true if send successful
     */
    bool setServoAngle(uint8_t servo_id, float angle) {
        /* Validate angle range */
        if (angle < 0.0f || angle > 180.0f) {
            std::cerr << "Error: Angle out of range [0, 180]" << std::endl;
            return false;
        }
        
        /* Build command string */
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(1) 
            << (int)servo_id << ":" << angle << "\r\n";
        
        std::string cmd = oss.str();
        
        try {
            /* Send command */
            size_t bytes_wrote = ser.write(cmd);
            
            if (bytes_wrote != cmd.length()) {
                std::cerr << "Error: Failed to send complete command" << std::endl;
                return false;
            }
            
            std::cout << "Sent: " << cmd << "(" << bytes_wrote << " bytes)" << std::endl;
            return true;
            
        } catch (serial::SerialException &e) {
            std::cerr << "Error sending command: " << e.what() << std::endl;
            return false;
        }
    }
    
    /**
     * @brief Read response message
     * @param timeout_ms Timeout in milliseconds
     * @return Received response string
     */
    std::string readResponse(uint32_t timeout_ms = 1000) {
        std::string response;
        
        if (!ser.isOpen()) {
            return "";
        }
        
        try {
            if (ser.waitReadable()) {
                size_t bytes_available = ser.available();
                if (bytes_available > 0) {
                    response = ser.read(bytes_available);
                    std::cout << "Received: " << response;
                }
            }
        } catch (serial::SerialException &e) {
            std::cerr << "Error reading response: " << e.what() << std::endl;
        }
        
        return response;
    }
    
    /**
     * @brief Close serial port
     */
    void close() {
        if (ser.isOpen()) {
            ser.close();
            std::cout << "Serial port closed" << std::endl;
        }
    }
    
    /**
     * @brief Check if serial port is open
     */
    bool isOpen() const {
        return ser.isOpen();
    }
    
    /**
     * @brief Destructor
     */
    ~ServoController() {
        close();
    }
};
```

### Usage Example

```cpp
#include <thread>
#include <chrono>

int main() {
    ServoController controller;
    
    /* Initialize serial port (adjust port for your system) */
    if (!controller.init("/dev/ttyUSB0")) {  // Linux
    // if (!controller.init("COM3")) {        // Windows
        std::cerr << "Failed to initialize serial port" << std::endl;
        return -1;
    }
    
    /* Example 1: Set servo to different angles */
    std::cout << "\n=== Example 1: Set servo to different angles ===" << std::endl;
    
    controller.setServoAngle(0, 0.0f);      // 0 degrees
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    controller.readResponse();
    
    controller.setServoAngle(0, 45.0f);     // 45 degrees
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    controller.readResponse();
    
    controller.setServoAngle(0, 90.0f);     // 90 degrees (middle position)
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    controller.readResponse();
    
    controller.setServoAngle(0, 135.0f);    // 135 degrees
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    controller.readResponse();
    
    controller.setServoAngle(0, 180.0f);    // 180 degrees
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    controller.readResponse();
    
    /* Example 2: Smooth sweep */
    std::cout << "\n=== Example 2: Smooth sweep ===" << std::endl;
    
    for (float angle = 0.0f; angle <= 180.0f; angle += 10.0f) {
        controller.setServoAngle(0, angle);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        controller.readResponse();
    }
    
    /* Example 3: Multiple servos */
    std::cout << "\n=== Example 3: Multiple servos ===" << std::endl;
    
    for (uint8_t servo_id = 0; servo_id < 3; servo_id++) {
        controller.setServoAngle(servo_id, 90.0f);  // Set all servos to 90 degrees
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        controller.readResponse();
    }
    
    /* Close serial port */
    controller.close();
    
    return 0;
}
```

### Simple C Implementation (Without External Libraries)

If you prefer pure C or a simpler approach without external libraries:

```c
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

int open_serial_port(const char *port) {
    int fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0) {
        perror("open_port_error");
        return -1;
    }
    
    struct termios options;
    tcgetattr(fd, &options);
    
    /* Set baud rate to 921600 */
    cfsetispeed(&options, B921600);
    cfsetospeed(&options, B921600);
    
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~PARENB;
    
    tcsetattr(fd, TCSANOW, &options);
    
    return fd;
}

void send_servo_command(int fd, uint8_t servo_id, float angle) {
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "%d:%.1f\r\n", servo_id, angle);
    
    ssize_t written = write(fd, cmd, strlen(cmd));
    if (written > 0) {
        printf("Sent: %s", cmd);
    } else {
        printf("Error sending command\n");
    }
}

int main() {
    int fd = open_serial_port("/dev/ttyUSB0");
    if (fd < 0) {
        return -1;
    }
    
    /* Set servo 0 to 90 degrees */
    send_servo_command(fd, 0, 90.0f);
    sleep(1);
    
    /* Set servo 0 to 45 degrees */
    send_servo_command(fd, 0, 45.0f);
    sleep(1);
    
    close(fd);
    return 0;
}
```

## Frequently Asked Questions

### Q1: How do I find the serial port number?

**Windows**:
- Open Device Manager and look for "COM ports"

**Linux**:
```bash
ls /dev/tty*
# or
dmesg | grep tty
```

**macOS**:
```bash
ls /dev/tty.*
```

### Q2: The baud rate 921600 seems too high. Can I change it?

You can modify the baud rate in the firmware, but you must adjust the host application to match. The firmware baud rate setting is in `usart.c`:

```c
huart1.Init.BaudRate = 921600;  // Modify this value
```

Common alternatives: 115200, 230400, 460800, 921600

### Q3: How can I verify that communication is working?

1. Use a serial terminal tool (such as PuTTY, TeraTerm, minicom)
2. Connect to the board at 921600 baud
3. Send a test command: `0:90.0\r\n`
4. Verify that the servo moves and you receive a response

### Q4: The command is not being executed. What should I check?

1. Verify the baud rate is set correctly
2. Confirm command format is correct: `servo_id:angle\r\n`
3. Ensure angle is within 0-180 range
4. Check that UART idle interrupt is properly triggered
5. Verify DMA is correctly configured

## Key Files

| File | Description |
|------|-------------|
| `Core/Inc/drv_comm.h` | Communication driver header file |
| `Core/Src/drv_comm.c` | Communication driver implementation |
| `Core/Inc/drv_servo.h` | Servo driver header file |
| `Core/Src/drv_servo.c` | Servo driver implementation |

## Build and Flash

```bash
cd cmake-build-debug
cmake ..
make -j$(nproc)

# Generated firmware files:
# - MRC_HE_CONTROL.elf
# - MRC_HE_CONTROL.hex
# - MRC_HE_CONTROL.bin
```

## Version Information

- **Creation Date**: 2026-01-31
- **MCU**: STM32F103RCT6
- **UART**: USART1, 921600 bps
- **PWM Frequency**: 50 Hz
- **Servo Range**: 0° - 180°
- **DMA Channels**: DMA1_Channel4 (TX), DMA1_Channel5 (RX)
