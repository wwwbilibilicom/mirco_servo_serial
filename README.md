# Mirco Servo Serial

A C++ library for controlling servo motors via serial communication without external dependencies.

## 📋 Features

- Object-oriented design, easy to integrate
- Supports multiple baud rates (9600 - 921600)
- Angle range validation (0° - 180°)
- Complete error handling mechanism
- No external dependencies, only standard C++ and POSIX API
- Provides both static and shared libraries

## 📁 Project Structure

```
mirco_servo_serial/
├── CMakeLists.txt          # CMake build configuration
├── README.md               # Project documentation
├── README_COMM.md          # Communication protocol documentation
├── include/
│   └── servo_controller.h  # ServoController class header
├── src/
│   └── servo_controller.cpp # ServoController class implementation
└── example/
    └── mirco_servo_serial.cpp # Usage example
```

## 🔧 Build

### Requirements

- CMake >= 3.10
- C++11 compatible compiler (GCC, Clang, etc.)

### Build Steps

```bash
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

### Debug Mode

```bash
cmake -DCMAKE_BUILD_TYPE=Debug ..
make
```

### Install (Optional)

```bash
sudo make install
```

## 📖 Usage

### Basic Usage

```cpp
#include "servo_controller.h"

int main() {
    ServoController controller;
    
    // Initialize serial port
    if (!controller.init("/dev/ttyUSB0", 921600)) {
        return -1;
    }
    
    // Set servo angles
    controller.setServoAngle(0, 90.0f);  // Servo 0 to 90 degrees
    controller.setServoAngle(1, 45.0f);  // Servo 1 to 45 degrees
    
    // Port will be automatically closed in destructor
    return 0;
}
```

### API Reference

| Method | Description |
|--------|-------------|
| `init(port, baudrate)` | Initialize and open serial port |
| `setServoAngle(id, angle)` | Set angle for specified servo |
| `isOpen()` | Check if serial port is open |
| `close()` | Close serial port |
| `getLastError()` | Get last error message |

### Linking

```cmake
# In your CMakeLists.txt
find_library(SERVO_CONTROLLER servo_controller)
target_link_libraries(your_target ${SERVO_CONTROLLER})
```

Or compile directly:

```bash
g++ -std=c++11 -I/path/to/include your_code.cpp -lservo_controller -o your_app
```

## 📡 Communication Protocol

Command format: `<servo_id>:<angle>\r\n`

| Parameter | Description | Range |
|-----------|-------------|-------|
| servo_id | Servo ID | 0-255 |
| angle | Target angle | 0.0 - 180.0 |

Examples:
```
0:90.0\r\n    # Servo 0 to 90 degrees
1:45.5\r\n    # Servo 1 to 45.5 degrees
```

For detailed protocol documentation, see [README_COMM.md](README_COMM.md)

## 📄 License

MIT License
