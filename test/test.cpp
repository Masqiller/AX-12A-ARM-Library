#include <Arduino.h>
#include <SoftwareSerial.h>

// Adjust pins for the board/shield wiring
// RX on Arduino listens to DXL line; TX drives the same line via direction control.
static const uint8_t DXL_RX_PIN = 8;   // to DYNAMIXEL DATA line (through shield)
static const uint8_t DXL_TX_PIN = 9;   // to DYNAMIXEL DATA line (through shield)
static const uint8_t DIR_PIN    = 2;   // DE/RE direction control (HIGH=TX, LOW=RX)
static const unsigned long DXL_BAUD = 115200; // target baud

SoftwareSerial dxl(DXL_RX_PIN, DXL_TX_PIN, true); // true = invert logic if shield requires; set to false otherwise

// Build and send a Protocol 1.0 Ping packet
void dxlSendPing(uint8_t id) {
  uint8_t hdr1 = 0xFF, hdr2 = 0xFF;
  uint8_t len  = 0x02;    // Length = params(0) + 2
  uint8_t inst = 0x01;    // Ping
  uint8_t sum  = ~(id + len + inst) & 0xFF;

  digitalWrite(DIR_PIN, HIGH);  // TX enable
  delayMicroseconds(2);

  dxl.write(hdr1);
  dxl.write(hdr2);
  dxl.write(id);
  dxl.write(len);
  dxl.write(inst);
  dxl.write(sum);
  dxl.flush();                  // ensure bytes out

  delayMicroseconds(10);
  digitalWrite(DIR_PIN, LOW);   // RX enable to read Status
}

// Read a Protocol 1.0 Status packet with a simple timeout
int dxlReadStatus(uint8_t &rx_id, uint8_t &err) {
  const uint32_t timeout_us = 2000; // 2 ms typical is enough at 115200
  uint32_t start = micros();

  // Find 0xFF 0xFF header
  int state = 0;
  while ((micros() - start) < timeout_us) {
    if (dxl.available()) {
      uint8_t b = dxl.read();
      if (state == 0 && b == 0xFF) { state = 1; }
      else if (state == 1 && b == 0xFF) { state = 2; break; }
      else { state = (b == 0xFF) ? 1 : 0; }
    }
  }
  if (state != 2) return -1; // no header

  // Read ID, LEN, ERR, then skip/consume to CHKSUM
  auto readByte = [&](uint8_t &out)->bool {
    uint32_t s = micros();
    while (!dxl.available()) {
      if ((micros() - s) > timeout_us) return false;
    }
    out = dxl.read();
    return true;
  };

  uint8_t id, len, error;
  if (!readByte(id))   return -2;
  if (!readByte(len))  return -3;
  if (!readByte(error))return -4;

  // Read parameters if any (len = params + 2)
  uint8_t params = (len >= 2) ? (len - 2) : 0;
  uint8_t discard;
  for (uint8_t i = 0; i < params; i++) {
    if (!readByte(discard)) return -5;
  }

  uint8_t chksum;
  if (!readByte(chksum)) return -6;

  rx_id = id;
  err   = error;
  return 0; // OK
}

void setup() {
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(DIR_PIN, LOW); // start in RX

  Serial.begin(115200);
  dxl.begin(DXL_BAUD);

  delay(50);
  Serial.println("Pinging AX-12A at 115200...");
  
  // Ping ID 1
  dxlSendPing(0x01);
  uint8_t rid=0, err=0;
  int res = dxlReadStatus(rid, err);
  if (res == 0) {
    Serial.print("Ping OK from ID ");
    Serial.print(rid);
    Serial.print(", ERR=0x");
    Serial.println(err, HEX);
  } else {
    Serial.print("Ping ID 1 failed, code "); Serial.println(res);
  }

  // Ping Broadcast 0xFE (some devices reply, though broadcast ping is not guaranteed to get replies)
  dxlSendPing(0xFE);
  res = dxlReadStatus(rid, err);
  if (res == 0) {
    Serial.print("Broadcast ping got reply from ID ");
    Serial.print(rid);
    Serial.print(", ERR=0x");
    Serial.println(err, HEX);
  } else {
    Serial.print("Broadcast ping: no reply (code "); Serial.print(res); Serial.println(")");
  }
}

void loop() {
  // nothing
}
