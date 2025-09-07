//##See the values here are capped at 444-819 ,you may change them according to your needs.##
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

static void dxlWriteSpeed(uint8_t id, uint16_t speed) {
  uint8_t p[3];
  p[0] = 32;                 // Start address (Moving Speed L)
  p[1] = (uint8_t)(speed & 0xFF);
  p[2] = (uint8_t)(speed >> 8);
  dxlWritePacket(id, 0x03, p, 3);
}

// Example: command -90, 0, +90 degrees on ID 1
static const uint8_t SERVO_ID = 1;

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
  dxlTorqueEnable(SERVO_ID, true);
  
  // Wait a bit for serial monitor to connect
  delay(2000);
  
  // Check if serial monitor is connected (by checking if there's input)
  Serial.println("Send any character to initialize servo to position 512, or send commands directly");
}

void loop() {
  if (Serial.available() > 0) {
    // Read the incoming string from serial monitor
    String input = Serial.readStringUntil('\n');
    input.trim(); // Remove any whitespace
    
    // Check if this is the first input after opening serial monitor
    static bool initialized = false;
    if (!initialized) {
      // Move to position 512 as requested
      dxlWriteGoal(SERVO_ID, 512);
      Serial.println("Servo initialized to position 512");
      Serial.println("Now ready for commands. Options:");
      Serial.println("1. Position (444-819): e.g., 512");  // Updated range
      Serial.println("2. Position and speed: e.g., 512,200");
      Serial.println("3. Angle mode: e.g., a45 or a45,200");
      initialized = true;
      
      // If the input was just for initialization, don't process it as a command
      if (input.length() == 1 && (input.charAt(0) == '1' || input.charAt(0) == '0')) {
        return;
      }
    }
    
    // Check if input starts with 'a' or 'A' for angle mode
    if (input.charAt(0) == 'a' || input.charAt(0) == 'A') {
      // Angle mode: a<angle> or a<angle>,<speed>
      input.remove(0, 1); // Remove the 'a' prefix
      
      if (input.indexOf(',') != -1) {
        // Split the input into angle and speed
        int commaIndex = input.indexOf(',');
        String angleStr = input.substring(0, commaIndex);
        String speedStr = input.substring(commaIndex + 1);
        
        float angle = angleStr.toFloat();
        uint16_t speed = speedStr.toInt();
        
        // Cap angle at -20 to +90 degrees
        if (angle < -20.0f) angle = -20.0f;
        if (angle > 90.0f) angle = 90.0f;
        
        // Validate speed
        if (speed >= 0 && speed <= 1023) {
          // Convert angle to position
          uint16_t position = degreesToGoal(angle);
          
          // Set speed first, then position
          dxlWriteSpeed(SERVO_ID, speed);
          dxlWriteGoal(SERVO_ID, position);
          
          // Send confirmation back to serial monitor
          Serial.print("Angle set to: ");
          Serial.print(angle);
          Serial.print(" deg (position ");
          Serial.print(position);
          Serial.print("), Speed set to: ");
          Serial.println(speed);
        } else {
          Serial.println("Invalid speed. Please use format: a<angle>,<speed> (speed 0-1023)");
        }
      } else {
        // Only angle provided
        float angle = input.toFloat();
        
        // Cap angle at -20 to +90 degrees
        if (angle < -20.0f) angle = -20.0f;
        if (angle > 90.0f) angle = 90.0f;
        
        uint16_t position = degreesToGoal(angle);
        
        // Send position to servo
        dxlWriteGoal(SERVO_ID, position);
        
        // Send confirmation back to serial monitor
        Serial.print("Angle set to: ");
        Serial.print(angle);
        Serial.print(" deg (position ");
        Serial.print(position);
        Serial.println(")");
        Serial.println("Speed unchanged");
      }
    } else if (input.indexOf(',') != -1) {
      // Position,speed format
      int commaIndex = input.indexOf(',');
      String positionStr = input.substring(0, commaIndex);
      String speedStr = input.substring(commaIndex + 1);
      
      uint16_t position = positionStr.toInt();
      uint16_t speed = speedStr.toInt();
      
      // Cap position at 444-819 (changed minimum from 342 to 444)
      if (position < 444) position = 444;
      if (position > 819) position = 819;
      
      // Validate position and speed
      if (position <= 1023 && speed >= 0 && speed <= 1023) {
        // Set speed first, then position
        dxlWriteSpeed(SERVO_ID, speed);
        dxlWriteGoal(SERVO_ID, position);
        
        // Send confirmation back to serial monitor
        Serial.print("Position set to: ");
        Serial.print(position);
        Serial.print(" (");
        Serial.print(goalToDegrees(position));
        Serial.print(" deg), Speed set to: ");
        Serial.println(speed);
      } else {
        Serial.println("Invalid input. Please use format: position,speed (position 444-819, speed 0-1023)");  // Updated range
      }
    } else {
      // Only position provided (backward compatibility)
      if (input.length() > 0 && input.toInt() >= 0) {
        uint16_t position = input.toInt();
        
        // Cap position at 444-819 (changed minimum from 342 to 444)
        if (position < 444) position = 444;
        if (position > 819) position = 819;
        
        // Send position to servo
        dxlWriteGoal(SERVO_ID, position);
        
        // Send confirmation back to serial monitor
        Serial.print("Position set to: ");
        Serial.print(position);
        Serial.print(" (");
        Serial.print(goalToDegrees(position));
        Serial.println(" deg)");
        Serial.println("Speed unchanged");
      } else {
        Serial.println("Invalid input. Options:");
        Serial.println("1. Position (444-819): e.g., 512");  // Updated range
        Serial.println("2. Position and speed: e.g., 512,200");
        Serial.println("3. Angle mode: e.g., a45 or a45,200");
      }
    }
  }
}
