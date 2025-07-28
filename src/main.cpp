#include <Arduino.h>

// Function prototypes
void setServoPosition(uint8_t servo_id, uint16_t position);
void setServoAngle(uint8_t servo_id, uint8_t angle);

void setup()
{
  Serial.begin(1000000); // Initialize serial communication at 1 Mbps

  // Move servos to specified angles (execute only once)
  // setServoAngle(1, 90);   // Servo 1 to 90°
  // setServoAngle(14, 180); // Servo 14 to 180°
  setServoPosition(14, 2000);
}

void loop()
{
  // Do nothing in the loop; the servos will stay in their positions
}

/**
 * @brief Sets the servo position based on a given angle.
 * 
 * @param servo_id The ID of the servo (1 to 254).
 * @param angle The desired angle in degrees (0 to 180).
 */
void setServoAngle(uint8_t servo_id, uint8_t angle)
{
  // Ensure the angle is within the valid range (0 to 180)
  if (angle > 300) {
    angle = 300; // Clamp the angle to the maximum value
  }

  // Map the angle (0-180) to the position range (0-1023)
  uint16_t position = map(angle, 0, 300, 0, 1023);

  // Call the setServoPosition function with the calculated position
  setServoPosition(servo_id, position);
}

/**
 * @brief Sends a command to set the servo position.
 * 
 * @param servo_id The ID of the servo (1 to 254).
 * @param position The desired position (0 to 1023).
 */
void setServoPosition(uint8_t servo_id, uint16_t position)
{
  // Step 1: Extract the lower and higher bytes from the position
  uint8_t lower_byte = position & 0xFF;         // Lower 8 bits
  uint8_t higher_byte = (position >> 8) & 0x03; // Upper 2 bits

  // Step 2: Define the parameters
  uint8_t param1 = 30; // Starting address
  uint8_t param2 = lower_byte;
  uint8_t param3 = higher_byte;

  // Step 3: Calculate the length
  uint8_t num_parameters = 3;          // Starting address, lower byte, higher byte
  uint8_t length = num_parameters + 2; // Length = number of parameters + 2 (Instruction + Checksum)

  // Step 4: Calculate the checksum
  uint16_t sum = servo_id + length + 3 + param1 + param2 + param3;
  uint8_t checksum = ~sum; // Binary ones complement

  // Step 5: Send the packet via Serial
  Serial.write(0xFF);     // Header
  Serial.write(0xFF);     // Header
  Serial.write(servo_id); // Servo ID
  Serial.write(length);   // Length
  Serial.write(3);        // Instruction
  Serial.write(param1);   // Starting address
  Serial.write(param2);   // Lower byte
  Serial.write(param3);   // Higher byte
  Serial.write(checksum); // Checksum
}