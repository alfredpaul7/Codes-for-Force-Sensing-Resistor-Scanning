//C++ code for 64x64 with SPI protocol but working needs to be checked
#include <Arduino.h>
#include <SPI.h>

// --- SPI Pin Definitions ---
const int slaveSelectPin = 10; 

// --- CD74HC4067 Address Pins ---
const int sPins[] = {2, 3, 4, 5};

// --- Analog Rows (8 Mux outputs connected to 8 Analog Pins) ---
// Sensors 1 & 2 (Upper half of the matrix)
const int rowsTop[] = {A0, A1, A2, A3}; 
// Sensors 3 & 4 (Lower half of the matrix)
const int rowsBottom[] = {A4, A5, A6, A7};

void setActiveColumn(int col);
void clearAllColumns();
void setMuxAddress(int ch);

void setup() {
  Serial.begin(921600);
  pinMode(slaveSelectPin, OUTPUT);
  digitalWrite(slaveSelectPin, HIGH);
  
  SPI.begin();
  // High speed SPI for faster scanning
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  
  for (int i = 0; i < 4; i++) {
    pinMode(sPins[i], OUTPUT);
  }
  clearAllColumns();
}

void loop() {
  for (int col = 0; col < 64; col++) {
    setActiveColumn(col);
    
    for (int ch = 0; ch < 16; ch++) {
      setMuxAddress(ch);
      delayMicroseconds(15); // Allow analog voltage to settle
      
      // Read Top Sensors (Rows 0-31)
      for (int r = 0; r < 4; r++) {
        Serial.print(analogRead(rowsTop[r]));
        Serial.print(",");
      }
      
      // Read Bottom Sensors (Rows 32-63)
      for (int r = 0; r < 4; r++) {
        Serial.print(analogRead(rowsBottom[r]));
        
        // Handle trailing comma: Only omit for the very last value of the 4096 set
        if (col == 63 && ch == 15 && r == 3) {
           // End of frame
        } else {
           Serial.print(",");
        }
      }
    }
  }
  Serial.println(); // Signal end of frame
}

void setMuxAddress(int ch) {
  digitalWrite(sPins[0], (ch & 0x01));
  digitalWrite(sPins[1], (ch >> 1) & 0x01);
  digitalWrite(sPins[2], (ch >> 2) & 0x01);
  digitalWrite(sPins[3], (ch >> 3) & 0x01);
}

void setActiveColumn(int col) {
  // Use 64-bit unsigned integer to represent 64 columns
  uint64_t bitmask = 1ULL << col; 
  
  digitalWrite(slaveSelectPin, LOW);
  // Transfer 8 bytes (64 bits) to the 16 shift registers
  for (int i = 7; i >= 0; i--) {
    SPI.transfer((bitmask >> (i * 8)) & 0xFF);
  }
  digitalWrite(slaveSelectPin, HIGH);
}

void clearAllColumns() {
  digitalWrite(slaveSelectPin, LOW);
  for(int i = 0; i < 8; i++) {
    SPI.transfer(0);
  }
  digitalWrite(slaveSelectPin, HIGH);
}
