#include <Arduino.h>

// Configure pins and baud
static const uint8_t DIR_PIN    = 2;        // Half-duplex direction control (HIGH=TX, LOW=RX)
static const uint8_t DXL_TX_PIN = 9;        // UART TX pin if using SoftwareSerial or Serial1 tx
static const uint8_t DXL_RX_PIN = 8;        // UART RX pin (not used if no readback)
static const unsigned long DXL_BAUD = 115200; // Current bus baud rate

// Select which Serial to use for DXL line
// For Uno/Nano: use SoftwareSerial or a TTL half-duplex adapter
// For boards with Serial1 (e.g., Mega, Leonardo, Micro): prefer Serial1
// #define USE_HW_SERIAL 1

#ifndef USE_HW_SERIAL
  #include <SoftwareSerial.h>
  SoftwareSerial dxl(DXL_RX_PIN, DXL_TX_PIN, false); // no inversion by default
  #define DXL_SERIAL dxl
#else
  #define DXL_SERIAL Serial1
#endif

// Protocol 1.0 helpers
static void dxlTxEnable(bool en) {
  digitalWrite(DIR_PIN, en ? HIGH : LOW);
  if (en) delayMicroseconds(2);
}

static uint8_t dxlChecksum(uint8_t id, uint8_t len, uint8_t inst, const uint8_t* params, uint8_t n) {
  uint16_t s = id + len + inst;
  for (uint8_t i = 0; i < n; i++) s += params[i];
  return (uint8_t)(~s);
}

static void dxlWritePacket(uint8_t id, uint8_t inst, const uint8_t* params, uint8_t nparams) {
  uint8_t len = 2 + nparams; // INST + PARAMS + ERR(virt)
  uint8_t cks = dxlChecksum(id, len, inst, params, nparams);

  dxlTxEnable(true);
  DXL_SERIAL.write(0xFF); DXL_SERIAL.write(0xFF);
  DXL_SERIAL.write(id);
  DXL_SERIAL.write(len);
  DXL_SERIAL.write(inst);
  for (uint8_t i = 0; i < nparams; i++) DXL_SERIAL.write(params[i]);
  DXL_SERIAL.write(cks);
  DXL_SERIAL.flush();
  delayMicroseconds(10);
  dxlTxEnable(false);
}

// High-level writes
static void dxlTorqueEnable(uint8_t id, bool on) {
  uint8_t p[2] = { 24, (uint8_t)(on ? 1 : 0) }; // Addr 24
  dxlWritePacket(id, 0x03, p, 2);
}

static void dxlWriteSpeed(uint8_t id, uint16_t speed) {
  uint8_t p[3];
  p[0] = 32;                 // Start address (Moving Speed L)
  p[1] = (uint8_t)(speed & 0xFF);
  p[2] = (uint8_t)(speed >> 8);
  dxlWritePacket(id, 0x03, p, 3);
}

static uint16_t degreesToGoal(float deg) {
  // Cap angle at -150 to +150 degrees (original range)
  if (deg < -150.0f) deg = -150.0f;
  if (deg > 150.0f) deg = 150.0f;
  
  // v = round(512 + deg * 1023/300), clamp 0..1023; −150..150 ≈ 0..1023
  float v = 512.0f + deg * (1023.0f/300.0f);
  if (v < 0) v = 0;
  if (v > 1023) v = 1023;
  return (uint16_t)lroundf(v);
}

static float goalToDegrees(uint16_t goal) {
  // deg = (goal - 512) * 300/1023, range approx -150..150
  if (goal > 1023) goal = 1023; // Cap at maximum
  return (goal - 512.0f) * (300.0f/1023.0f);
}

static void dxlWriteGoal(uint8_t id, uint16_t goal) {
  uint8_t p[3];
  p[0] = 30;                 // Start address (Goal Position L)
  p[1] = (uint8_t)(goal & 0xFF);
  p[2] = (uint8_t)(goal >> 8);
  dxlWritePacket(id, 0x03, p, 3);
}

// Servo IDs
static const uint8_t SERVO_ID_1 = 1;
static const uint8_t SERVO_ID_8 = 8;

void setup() {
  pinMode(DIR_PIN, OUTPUT);
  dxlTxEnable(false);

  Serial.begin(115200);
#ifdef USE_HW_SERIAL
  DXL_SERIAL.begin(DXL_BAUD);
#else
  DXL_SERIAL.begin(DXL_BAUD);
#endif

  // Enable torque but don't move to any specific position at startup
  dxlTorqueEnable(SERVO_ID_1, true);
  dxlTorqueEnable(SERVO_ID_8, true);
  
  // Wait a bit for serial monitor to connect
  delay(2000);
  
  // Display help message
  Serial.println("Servo Control Ready. Commands:");
  Serial.println("1. Initialize both servos: Send any character");
  Serial.println("2. Control specific servo:");
  Serial.println("   - s1,<position>[,<speed>]  (Servo 1)");
  Serial.println("   - s8,<position>[,<speed>]  (Servo 8)");
  Serial.println("   - s1a,<angle>[,<speed>]    (Servo 1, angle mode)");
  Serial.println("   - s8a,<angle>[,<speed>]    (Servo 8, angle mode)");
  Serial.println("3. Control both servos:");
  Serial.println("   - b,<position>[,<speed>]   (Both servos, same params)");
  Serial.println("   - ba,<angle>[,<speed>]     (Both servos, same params)");
  Serial.println("   - d,<pos1>,<pos2>,<speed1>,<speed2> (Different params)");
  Serial.println("   - da,<ang1>,<ang2>,<speed1>,<speed2> (Different params)");
  Serial.println("Position range: 444-819, Angle range: -20 to +90, Speed range: 0-1023");
}

void loop() {
  if (Serial.available() > 0) {
    // Read the incoming string from serial monitor
    String input = Serial.readStringUntil('\n');
    input.trim(); // Remove any whitespace
    
    // Check if this is the first input after opening serial monitor
    static bool initialized = false;
    if (!initialized) {
      // Move both servos to position 512 as requested
      dxlWriteGoal(SERVO_ID_1, 512);
      dxlWriteGoal(SERVO_ID_8, 512);
      Serial.println("Servos initialized to position 512");
      initialized = true;
      return;
    }
    
    // Parse command
    if (input.startsWith("s1,")) {
      // Control Servo 1 with position
      input.remove(0, 3); // Remove "s1,"
      int commaIndex = input.indexOf(',');
      if (commaIndex != -1) {
        // Position and speed
        String positionStr = input.substring(0, commaIndex);
        String speedStr = input.substring(commaIndex + 1);
        uint16_t position = positionStr.toInt();
        uint16_t speed = speedStr.toInt();
        
        // Cap position at 444-819
        if (position < 444) position = 444;
        if (position > 819) position = 819;
        
        // Validate speed
        if (speed <= 1023) {
          dxlWriteSpeed(SERVO_ID_1, speed);
          dxlWriteGoal(SERVO_ID_1, position);
          Serial.print("Servo 1: Position=");
          Serial.print(position);
          Serial.print(" (");
          Serial.print(goalToDegrees(position));
          Serial.print(" deg) Speed=");
          Serial.println(speed);
        } else {
          Serial.println("Invalid speed for Servo 1. Range: 0-1023");
        }
      } else {
        // Only position
        uint16_t position = input.toInt();
        
        // Cap position at 444-819
        if (position < 444) position = 444;
        if (position > 819) position = 819;
        
        dxlWriteGoal(SERVO_ID_1, position);
        Serial.print("Servo 1: Position=");
        Serial.print(position);
        Serial.print(" (");
        Serial.print(goalToDegrees(position));
        Serial.println(" deg)");
      }
    } else if (input.startsWith("s8,")) {
      // Control Servo 8 with position
      input.remove(0, 3); // Remove "s8,"
      int commaIndex = input.indexOf(',');
      if (commaIndex != -1) {
        // Position and speed
        String positionStr = input.substring(0, commaIndex);
        String speedStr = input.substring(commaIndex + 1);
        uint16_t position = positionStr.toInt();
        uint16_t speed = speedStr.toInt();
        
        // Cap position at 444-819
        if (position < 444) position = 444;
        if (position > 819) position = 819;
        
        // Validate speed
        if (speed <= 1023) {
          dxlWriteSpeed(SERVO_ID_8, speed);
          dxlWriteGoal(SERVO_ID_8, position);
          Serial.print("Servo 8: Position=");
          Serial.print(position);
          Serial.print(" (");
          Serial.print(goalToDegrees(position));
          Serial.print(" deg) Speed=");
          Serial.println(speed);
        } else {
          Serial.println("Invalid speed for Servo 8. Range: 0-1023");
        }
      } else {
        // Only position
        uint16_t position = input.toInt();
        
        // Cap position at 444-819
        if (position < 444) position = 444;
        if (position > 819) position = 819;
        
        dxlWriteGoal(SERVO_ID_8, position);
        Serial.print("Servo 8: Position=");
        Serial.print(position);
        Serial.print(" (");
        Serial.print(goalToDegrees(position));
        Serial.println(" deg)");
      }
    } else if (input.startsWith("s1a,")) {
      // Control Servo 1 with angle
      input.remove(0, 4); // Remove "s1a,"
      int commaIndex = input.indexOf(',');
      if (commaIndex != -1) {
        // Angle and speed
        String angleStr = input.substring(0, commaIndex);
        String speedStr = input.substring(commaIndex + 1);
        float angle = angleStr.toFloat();
        uint16_t speed = speedStr.toInt();
        
        // Cap angle at -20 to +90 degrees
        if (angle < -20.0f) angle = -20.0f;
        if (angle > 90.0f) angle = 90.0f;
        
        uint16_t position = degreesToGoal(angle);
        
        // Validate speed
        if (speed <= 1023) {
          dxlWriteSpeed(SERVO_ID_1, speed);
          dxlWriteGoal(SERVO_ID_1, position);
          Serial.print("Servo 1: Angle=");
          Serial.print(angle);
          Serial.print(" deg (Position=");
          Serial.print(position);
          Serial.print(") Speed=");
          Serial.println(speed);
        } else {
          Serial.println("Invalid speed for Servo 1. Range: 0-1023");
        }
      } else {
        // Only angle
        float angle = input.toFloat();
        
        // Cap angle at -20 to +90 degrees
        if (angle < -20.0f) angle = -20.0f;
        if (angle > 90.0f) angle = 90.0f;
        
        uint16_t position = degreesToGoal(angle);
        dxlWriteGoal(SERVO_ID_1, position);
        Serial.print("Servo 1: Angle=");
        Serial.print(angle);
        Serial.print(" deg (Position=");
        Serial.print(position);
        Serial.println(")");
      }
    } else if (input.startsWith("s8a,")) {
      // Control Servo 8 with angle
      input.remove(0, 4); // Remove "s8a,"
      int commaIndex = input.indexOf(',');
      if (commaIndex != -1) {
        // Angle and speed
        String angleStr = input.substring(0, commaIndex);
        String speedStr = input.substring(commaIndex + 1);
        float angle = angleStr.toFloat();
        uint16_t speed = speedStr.toInt();
        
        // Cap angle at -20 to +90 degrees
        if (angle < -20.0f) angle = -20.0f;
        if (angle > 90.0f) angle = 90.0f;
        
        uint16_t position = degreesToGoal(angle);
        
        // Validate speed
        if (speed <= 1023) {
          dxlWriteSpeed(SERVO_ID_8, speed);
          dxlWriteGoal(SERVO_ID_8, position);
          Serial.print("Servo 8: Angle=");
          Serial.print(angle);
          Serial.print(" deg (Position=");
          Serial.print(position);
          Serial.print(") Speed=");
          Serial.println(speed);
        } else {
          Serial.println("Invalid speed for Servo 8. Range: 0-1023");
        }
      } else {
        // Only angle
        float angle = input.toFloat();
        
        // Cap angle at -20 to +90 degrees
        if (angle < -20.0f) angle = -20.0f;
        if (angle > 90.0f) angle = 90.0f;
        
        uint16_t position = degreesToGoal(angle);
        dxlWriteGoal(SERVO_ID_8, position);
        Serial.print("Servo 8: Angle=");
        Serial.print(angle);
        Serial.print(" deg (Position=");
        Serial.print(position);
        Serial.println(")");
      }
    } else if (input.startsWith("b,")) {
      // Control both servos with position
      input.remove(0, 2); // Remove "b,"
      int commaIndex = input.indexOf(',');
      if (commaIndex != -1) {
        // Position and speed
        String positionStr = input.substring(0, commaIndex);
        String speedStr = input.substring(commaIndex + 1);
        uint16_t position = positionStr.toInt();
        uint16_t speed = speedStr.toInt();
        
        // Cap position at 444-819
        if (position < 444) position = 444;
        if (position > 819) position = 819;
        
        // Validate speed
        if (speed <= 1023) {
          dxlWriteSpeed(SERVO_ID_1, speed);
          dxlWriteSpeed(SERVO_ID_8, speed);
          dxlWriteGoal(SERVO_ID_1, position);
          dxlWriteGoal(SERVO_ID_8, position);
          Serial.print("Both Servos: Position=");
          Serial.print(position);
          Serial.print(" (");
          Serial.print(goalToDegrees(position));
          Serial.print(" deg) Speed=");
          Serial.println(speed);
        } else {
          Serial.println("Invalid speed for both servos. Range: 0-1023");
        }
      } else {
        // Only position
        uint16_t position = input.toInt();
        
        // Cap position at 444-819
        if (position < 444) position = 444;
        if (position > 819) position = 819;
        
        dxlWriteGoal(SERVO_ID_1, position);
        dxlWriteGoal(SERVO_ID_8, position);
        Serial.print("Both Servos: Position=");
        Serial.print(position);
        Serial.print(" (");
        Serial.print(goalToDegrees(position));
        Serial.println(" deg)");
      }
    } else if (input.startsWith("ba,")) {
      // Control both servos with angle
      input.remove(0, 3); // Remove "ba,"
      int commaIndex = input.indexOf(',');
      if (commaIndex != -1) {
        // Angle and speed
        String angleStr = input.substring(0, commaIndex);
        String speedStr = input.substring(commaIndex + 1);
        float angle = angleStr.toFloat();
        uint16_t speed = speedStr.toInt();
        
        // Cap angle at -20 to +90 degrees
        if (angle < -20.0f) angle = -20.0f;
        if (angle > 90.0f) angle = 90.0f;
        
        uint16_t position = degreesToGoal(angle);
        
        // Validate speed
        if (speed <= 1023) {
          dxlWriteSpeed(SERVO_ID_1, speed);
          dxlWriteSpeed(SERVO_ID_8, speed);
          dxlWriteGoal(SERVO_ID_1, position);
          dxlWriteGoal(SERVO_ID_8, position);
          Serial.print("Both Servos: Angle=");
          Serial.print(angle);
          Serial.print(" deg (Position=");
          Serial.print(position);
          Serial.print(") Speed=");
          Serial.println(speed);
        } else {
          Serial.println("Invalid speed for both servos. Range: 0-1023");
        }
      } else {
        // Only angle
        float angle = input.toFloat();
        
        // Cap angle at -20 to +90 degrees
        if (angle < -20.0f) angle = -20.0f;
        if (angle > 90.0f) angle = 90.0f;
        
        uint16_t position = degreesToGoal(angle);
        dxlWriteGoal(SERVO_ID_1, position);
        dxlWriteGoal(SERVO_ID_8, position);
        Serial.print("Both Servos: Angle=");
        Serial.print(angle);
        Serial.print(" deg (Position=");
        Serial.print(position);
        Serial.println(")");
      }
    } else if (input.startsWith("d,")) {
      // Control both servos with different positions and speeds
      input.remove(0, 2); // Remove "d,"
      
      // Parse: pos1,pos2,speed1,speed2
      int firstComma = input.indexOf(',');
      int secondComma = input.indexOf(',', firstComma + 1);
      int thirdComma = input.indexOf(',', secondComma + 1);
      
      if (firstComma != -1 && secondComma != -1 && thirdComma != -1) {
        String pos1Str = input.substring(0, firstComma);
        String pos2Str = input.substring(firstComma + 1, secondComma);
        String speed1Str = input.substring(secondComma + 1, thirdComma);
        String speed2Str = input.substring(thirdComma + 1);
        
        uint16_t pos1 = pos1Str.toInt();
        uint16_t pos2 = pos2Str.toInt();
        uint16_t speed1 = speed1Str.toInt();
        uint16_t speed2 = speed2Str.toInt();
        
        // Cap positions at 444-819
        if (pos1 < 444) pos1 = 444;
        if (pos1 > 819) pos1 = 819;
        if (pos2 < 444) pos2 = 444;
        if (pos2 > 819) pos2 = 819;
        
        // Validate speeds
        if (speed1 <= 1023 && speed2 <= 1023) {
          // Set speed for each servo with a delay between commands
          dxlWriteSpeed(SERVO_ID_1, speed1);
          delay(5); // Small delay for communication stability
          dxlWriteSpeed(SERVO_ID_8, speed2);
          delay(5); // Small delay for communication stability
          
          // Set position for each servo with a delay between commands
          dxlWriteGoal(SERVO_ID_1, pos1);
          delay(5); // Small delay for communication stability
          dxlWriteGoal(SERVO_ID_8, pos2);
          
          Serial.print("Servo 1: Position=");
          Serial.print(pos1);
          Serial.print(" (");
          Serial.print(goalToDegrees(pos1));
          Serial.print(" deg) Speed=");
          Serial.println(speed1);
          Serial.print("Servo 8: Position=");
          Serial.print(pos2);
          Serial.print(" (");
          Serial.print(goalToDegrees(pos2));
          Serial.print(" deg) Speed=");
          Serial.println(speed2);
        } else {
          Serial.println("Invalid speeds. Range: 0-1023");
        }
      } else {
        Serial.println("Invalid format for 'd' command. Use: d,<pos1>,<pos2>,<speed1>,<speed2>");
      }
    } else if (input.startsWith("da,")) {
      // Control both servos with different angles and speeds
      input.remove(0, 3); // Remove "da,"
      
      // Parse: ang1,ang2,speed1,speed2
      int firstComma = input.indexOf(',');
      int secondComma = input.indexOf(',', firstComma + 1);
      int thirdComma = input.indexOf(',', secondComma + 1);
      
      if (firstComma != -1 && secondComma != -1 && thirdComma != -1) {
        String ang1Str = input.substring(0, firstComma);
        String ang2Str = input.substring(firstComma + 1, secondComma);
        String speed1Str = input.substring(secondComma + 1, thirdComma);
        String speed2Str = input.substring(thirdComma + 1);
        
        float ang1 = ang1Str.toFloat();
        float ang2 = ang2Str.toFloat();
        uint16_t speed1 = speed1Str.toInt();
        uint16_t speed2 = speed2Str.toInt();
        
        // Cap angles at -20 to +90 degrees
        if (ang1 < -20.0f) ang1 = -20.0f;
        if (ang1 > 90.0f) ang1 = 90.0f;
        if (ang2 < -20.0f) ang2 = -20.0f;
        if (ang2 > 90.0f) ang2 = 90.0f;
        
        uint16_t pos1 = degreesToGoal(ang1);
        uint16_t pos2 = degreesToGoal(ang2);
        
        // Validate speeds
        if (speed1 <= 1023 && speed2 <= 1023) {
          // Set speed for each servo with a delay between commands
          dxlWriteSpeed(SERVO_ID_1, speed1);
          delay(5); // Small delay for communication stability
          dxlWriteSpeed(SERVO_ID_8, speed2);
          delay(5); // Small delay for communication stability
          
          // Set position for each servo with a delay between commands
          dxlWriteGoal(SERVO_ID_1, pos1);
          delay(5); // Small delay for communication stability
          dxlWriteGoal(SERVO_ID_8, pos2);
          
          Serial.print("Servo 1: Angle=");
          Serial.print(ang1);
          Serial.print(" deg (Position=");
          Serial.print(pos1);
          Serial.print(") Speed=");
          Serial.println(speed1);
          Serial.print("Servo 8: Angle=");
          Serial.print(ang2);
          Serial.print(" deg (Position=");
          Serial.print(pos2);
          Serial.print(") Speed=");
          Serial.println(speed2);
        } else {
          Serial.println("Invalid speeds. Range: 0-1023");
        }
      } else {
        Serial.println("Invalid format for 'da' command. Use: da,<ang1>,<ang2>,<speed1>,<speed2>");
      }
    } else {
      Serial.println("Invalid command. Send 'h' for help.");
    }
  }
}
