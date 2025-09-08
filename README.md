# AX-12A ARM Servo Control Project

This project provides control interfaces for Dynamixel AX-12A servo motors using an Arduino platform. It implements the Dynamixel Protocol 1.0 for precise position and speed control of robotic servos.

## Project Overview

The AX-12A is a smart servo motor with built-in control electronics that allows for precise position, speed, and torque control. This project enables communication with these servos through an Arduino microcontroller using the half-duplex UART communication protocol.

### Key Features
- Position control with degree-to-position unit conversion
- Speed control for smooth servo movements
- Torque enable/disable functionality
- Support for both SoftwareSerial and hardware Serial communication
- Multiple test examples for different control scenarios
- (for getting id you will need ttl half duplex combiners and lots of things )

## Project Structure

```
AX-12A ARM/
├── include/          # Header files (currently empty)
├── lib/              # External libraries (currently empty)
├── src/              # Main source code
│   └── main.cpp      # Primary application code
├── test/             # Test examples and variations
│   ├── angle_speedpos.cpp     # Angle-based control with speed adjustment
│   ├── multi_servopos.cpp     # Multi-servo scanning and control
│   ├── nospeed_servopos.cpp   # Position control without speed setting
│   ├── speed_servo.cpp        # Speed and position control with serial interface
│   ├── test.cpp               # Ping functionality test
│   ├── test0.cpp              # Basic position control
│   ├── test2.cpp              # Basic position control (duplicate)
│   ├── test3working.cpp       # Working position control example
│   └── README                 # Test directory information
├── platformio.ini    # PlatformIO project configuration
└── README.md         # This file
```

## Hardware Requirements

- Arduino board (Uno, Nano, Mega, etc.)
- AX-12A Dynamixel servo(s)
- TTL communication interface or level shifter
- Half-duplex communication circuit (DE/RE control)
- 12V power supply for servos

### Pin Configuration
- **DXL_TX_PIN (Pin 9)**: Transmits data to the servo
- **DXL_RX_PIN (Pin 8)**: Receives data from the servo
- **DIR_PIN (Pin 2)**: Controls data direction (HIGH=TX, LOW=RX)

## Test Files Description

### 1. angle_speedpos.cpp
Advanced control example that allows:
- Position control using angle values (degrees)
- Speed adjustment for movements
- Serial command interface with multiple input formats:
  - Position only: `512`
  - Position and speed: `512,200`
  - Angle mode: `a45` or `a45,200`

### 2. multi_servopos.cpp
Scans the Dynamixel bus to detect connected servos by sending ping commands to IDs 1-253 and reporting which ones respond.

### 3. nospeed_servopos.cpp
Basic position control without speed setting. Accepts position values (0-1023) via serial and moves the servo to those positions.

### 4. speed_servo.cpp
Position and speed control with serial interface. Accepts:
- Position only: `512`
- Position and speed: `512,200`

### 5. test.cpp
Implements ping functionality to test servo communication. Sends ping packets to specific servo IDs and listens for responses.

### 6. test0.cpp
Basic position control example that demonstrates direct position setting without serial interface.

### 7. test2.cpp
Duplicate of test0.cpp with the same basic position control functionality.

### 8. test3working.cpp
Working example of sequential position control that moves the servo through predefined positions with delays.

## Usage

### Basic Position Control
The servos can be controlled using position values from 0-1023 where:
- 0: Fully counterclockwise position
- 512: Center position
- 1023: Fully clockwise position

### Angle-Based Control
For more intuitive control, angles can be used:
- -150° to +150° range
- 0° corresponds to position 512
- Conversion formula: `position = 512 + angle × (1023/300)`

### Speed Control
Servo speed can be controlled with values from 0-1023:
- 0: Minimum speed
- 1023: Maximum speed

## Communication Protocol

The project implements Dynamixel Protocol 1.0 with the following packet structure:
```
[0xFF][0xFF][ID][LENGTH][INSTRUCTION][PARAMETERS][CHECKSUM]
```

### Key Registers
- **Register 30**: Goal Position (2 bytes)
- **Register 32**: Moving Speed (2 bytes)
- **Register 24**: Torque Enable (1 byte)

## Building and Uploading

This project uses PlatformIO for building and uploading. Install PlatformIO and use the following commands:

```bash
# Build the project
pio run

# Upload to Arduino
pio run --target upload

# Upload and monitor serial output
pio run --target upload && pio device monitor
```

## Configuration

The main configuration parameters can be adjusted in each file:
- `SERVO_ID`: Target servo ID (default: 1)
- `DXL_BAUD`: Communication baud rate (default: 115200)
- Pin assignments for communication

For hardware serial support (more reliable at higher baud rates), uncomment:
```cpp
#define USE_HW_SERIAL 1
```

## Troubleshooting

### Common Issues
1. **Servo not responding**: Check power supply, wiring, and baud rate settings
2. **Erratic movements**: Ensure stable power supply and check for electrical noise
3. **Communication failures**: Verify pin connections and DE/RE control

### Debugging Tips
- Use the ping test to verify servo communication
- Monitor serial output for error messages
- Check that servo IDs are correctly configured
- Ensure the servo baud rate matches the code settings

## References

- [Dynamixel Protocol 1.0 Documentation](https://emanual.robotis.com/docs/en/dxl/protocol1/)

- [AX-12A Specifications](https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/)
