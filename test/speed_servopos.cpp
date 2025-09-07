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
  // v = round(512 + deg * 1023/300), clamp 0..1023; −90..90 ≈ 205..819
  float v = 512.0f + deg * (1023.0f/300.0f);
  if (v < 0) v = 0;
  if (v > 1023) v = 1023;
  return (uint16_t)lroundf(v);
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

  // Optional: ensure no responses are sent by the servo
  // uint8_t p_sr[6] = { 16, 0 }; // Status Return Level = 0 at addr 16 (EEPROM)
  // dxlWritePacket(SERVO_ID, 0x03, p_sr, 2);

  dxlTorqueEnable(SERVO_ID, true);
  uint8_t p_sr[3] = { 32, 0x64, 0x00 };
  dxlWritePacket(SERVO_ID, 0x03, p_sr, 3);

  uint16_t g1 = degreesToGoal(-60.0f); // ~205
  uint16_t g2 = degreesToGoal(0.0f);   // 512
  uint16_t g3 = degreesToGoal(60.0f);  // ~819

  dxlWriteGoal(SERVO_ID, g1);
  delay(2000);
  dxlWriteGoal(SERVO_ID, g2);
  delay(2000);
  dxlWriteGoal(SERVO_ID, g3);
  delay(2000);
}

void loop() {
  if (Serial.available() > 0) {
    // Read the incoming string from serial monitor
    String input = Serial.readStringUntil('\n');
    input.trim(); // Remove any whitespace
    
    // Check if input contains a comma (position,speed format)
    if (input.indexOf(',') != -1) {
      // Split the input into position and speed
      int commaIndex = input.indexOf(',');
      String positionStr = input.substring(0, commaIndex);
      String speedStr = input.substring(commaIndex + 1);
      
      uint16_t position = positionStr.toInt();
      uint16_t speed = speedStr.toInt();
      
      // Validate position and speed
      if (position >= 0 && position <= 1023 && speed >= 0 && speed <= 1023) {
        // Set speed first, then position
        dxlWriteSpeed(SERVO_ID, speed);
        dxlWriteGoal(SERVO_ID, position);
        
        // Send confirmation back to serial monitor
        Serial.print("Position set to: ");
        Serial.print(position);
        Serial.print(", Speed set to: ");
        Serial.println(speed);
      } else {
        Serial.println("Invalid input. Please use format: position,speed (0-1023 for both)");
      }
    } else {
      // Only position provided (backward compatibility)
      if (input.length() > 0 && input.toInt() >= 0 && input.toInt() <= 1023) {
        uint16_t position = input.toInt();
        
        // Send position to servo
        dxlWriteGoal(SERVO_ID, position);
        
        // Send confirmation back to serial monitor
        Serial.print("Position set to: ");
        Serial.println(position);
        Serial.println("Speed unchanged");
      } else {
        Serial.println("Invalid input. Please enter a value between 0 and 1023, or use format: position,speed");
      }
    }
  }
}
