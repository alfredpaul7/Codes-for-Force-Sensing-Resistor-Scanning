//C++ code for 32x32 with SPI protocol
#include <Arduino.h>
#include <SPI.h>
// --- SPI Pin Definitions ---
/*RCLK D10
SRCLK  D13
SER D11*/
const int slaveSelectPin = 10;  
// --- CD74HC4067 Pin Definitions ---
const int sPins[] = {2, 3, 4, 5};
const int rowA = A0; 
const int rowB = A1; 
void setActiveColumn(int col);
void clearAllColumns();
void setup() {
  // Use 921600 to match the fast SPI data collection
  Serial.begin(921600);
  // Initialize SPI
  pinMode(slaveSelectPin, OUTPUT);
  digitalWrite(slaveSelectPin, HIGH);
  SPI.begin();
  // Set SPI settings: 8MHz clock, MSB first, Mode 0
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  for (int i = 0; i < 4; i++) {
    pinMode(sPins[i], OUTPUT);
  }
  clearAllColumns();
}
void loop() {
  for (int col = 0; col < 32; col++) {
    setActiveColumn(col);
    for (int ch = 0; ch < 16; ch++) {
      // Set MUX address
      digitalWrite(sPins[0], (ch & 0x01));
      digitalWrite(sPins[1], (ch >> 1) & 0x01);
      digitalWrite(sPins[2], (ch >> 2) & 0x01);
      digitalWrite(sPins[3], (ch >> 3) & 0x01);
      delayMicroseconds(20); // Slightly reduced settling time
      int val1 = analogRead(rowA);
      int val2 = analogRead(rowB);
      Serial.print(val1);
      Serial.print(",");
      Serial.print(val2);
      if (ch < 15 || col < 31) {
        Serial.print(",");
      }
    }
  }
  Serial.println();
}
void setActiveColumn(int col) {
  // We need to send 4 bytes (32 bits)
  // Calculate which byte (0-3) and which bit (0-7) the column is in
  uint32_t bitmask = 0x00000001UL << col;
  digitalWrite(slaveSelectPin, LOW);
  // Send 4 bytes: Shift out 32 bits total
  SPI.transfer((bitmask >> 24) & 0xFF);
  SPI.transfer((bitmask >> 16) & 0xFF);
  SPI.transfer((bitmask >> 8) & 0xFF);
  SPI.transfer(bitmask & 0xFF);  
  digitalWrite(slaveSelectPin, HIGH);
}
void clearAllColumns() {
  digitalWrite(slaveSelectPin, LOW);
  for(int i=0; i<4; i++) SPI.transfer(0);
  digitalWrite(slaveSelectPin, HIGH);
}
