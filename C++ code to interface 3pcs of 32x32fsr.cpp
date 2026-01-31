#include <Arduino.h>
#include <SPI.h>

const int slaveSelectPin = 10; 
const int sPins[] = {2, 3, 4, 5};
void setActiveColumn(int col);
void clearAllColumns();
void setMuxAddress(int ch);
// Analog Rows for 3 Sensors
const int rowA = A0; const int rowB = A1; // Sensor 1 (Left)
const int rowC = A2; const int rowD = A3; // Sensor 2 (Middle)
const int rowE = A4; const int rowF = A5; // Sensor 3 (Right)

void setup() {
  Serial.begin(921600);
  pinMode(slaveSelectPin, OUTPUT);
  digitalWrite(slaveSelectPin, HIGH);
  SPI.begin();
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  for (int i = 0; i < 4; i++) pinMode(sPins[i], OUTPUT);
  
  // Clear all 96 columns initially
  digitalWrite(slaveSelectPin, LOW);
  for(int i=0; i<12; i++) SPI.transfer(0); 
  digitalWrite(slaveSelectPin, HIGH);
}

void loop() {
  // We iterate through 32 columns. Each "column" trigger activates 
  // one column on ALL THREE sensors simultaneously.
  for (int col = 0; col < 32; col++) {
    setActiveColumn(col);
    
    for (int ch = 0; ch < 16; ch++) {
      setMuxAddress(ch);
      delayMicroseconds(15); // Slight delay for ADC stability
      
      // Sensor 1
      Serial.print(analogRead(rowA)); Serial.print(",");
      Serial.print(analogRead(rowB)); Serial.print(",");
      // Sensor 2
      Serial.print(analogRead(rowC)); Serial.print(",");
      Serial.print(analogRead(rowD)); Serial.print(",");
      // Sensor 3
      Serial.print(analogRead(rowE)); Serial.print(",");
      Serial.print(analogRead(rowF));
      
      // Trailing comma management for a total of 3072 values (32*16*6)
      if (ch < 15 || col < 31) {
        Serial.print(",");
      }
    }
  }
  Serial.println(); 
}

void setMuxAddress(int ch) {
  for (int i = 0; i < 4; i++) {
    digitalWrite(sPins[i], (ch >> i) & 0x01);
  }
}

void setActiveColumn(int col) {
  // We need to send 12 bytes (96 bits). 
  // Each 4 bytes (32 bits) represents one 32x32 sensor tile.
  uint32_t bitmask = 0x00000001UL << col;
  
  digitalWrite(slaveSelectPin, LOW);
  // Send 3 identical bitmasks to activate the same column index on all 3 tiles
  for(int i=0; i<3; i++) {
    SPI.transfer((bitmask >> 24) & 0xFF);
    SPI.transfer((bitmask >> 16) & 0xFF);
    SPI.transfer((bitmask >> 8) & 0xFF);
    SPI.transfer(bitmask & 0xFF);
  }
  digitalWrite(slaveSelectPin, HIGH);
}
