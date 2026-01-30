 //code to get the values such has V_out ,Conducatance, R_fsr. etc 
#include <Arduino.h>

// --- Pin Definitions ---
const int ser    = 11; 
const int srclk  = 13; 
const int rclk   = 10; 
const int sPins[] = {2, 3, 4, 5};
const int rowB    = A1; // Reading from MUX 2

// --- Calibration Constants ---
const float VCC = 3.3;      // Supply voltage
const float R_FIXED = 10000.0; // 10k ohms fixed resistor

void setup() {
  Serial.begin(115200);
  pinMode(ser, OUTPUT);
  pinMode(srclk, OUTPUT);
  pinMode(rclk, OUTPUT);
  for (int i = 0; i < 4; i++) {
    pinMode(sPins[i], OUTPUT);
  }

  // --- 1. ACTIVATE SHIFT REGISTER 1, Qa ---
  digitalWrite(rclk, LOW);
  for (int i = 1; i <= 32; i++) {
    digitalWrite(srclk, LOW);
    if (i == 32) { 
      digitalWrite(ser, HIGH); 
    } else {
      digitalWrite(ser, LOW);
    }
    digitalWrite(srclk, HIGH);
  }
  digitalWrite(rclk, HIGH);

  // --- 2. SET MUX 2 TO CHANNEL 15 ---
  digitalWrite(sPins[0], HIGH); 
  digitalWrite(sPins[1], HIGH); 
  digitalWrite(sPins[2], HIGH); 
  digitalWrite(sPins[3], HIGH);

  // Print Header for Tabulation
  Serial.println("\n--- RoxiFSR Calibration Data ---");
  Serial.println("ADC\tV_out(V)\tR_fsr(Ohms)\tG(uS)");
  Serial.println("----------------------------------------------");
}

void loop() {
  delayMicroseconds(30);
  
  // 1. Read Raw ADC
  int adcVal = analogRead(rowB);

  // 2. Calculate V_out
  float vOut = (adcVal * VCC) / 4096.0;

  // 3. Calculate R_fsr (with safety check to avoid division by zero)
  float rFsr;
  float conductance;

  if (adcVal <= 5) { // If ADC is near zero, resistance is effectively infinite
    rFsr = 0; 
    conductance = 0;
  } else {
    rFsr = ((VCC - vOut) * R_FIXED) / vOut;
    // 4. Calculate Conductance (G) in microSiemens (uS)
    conductance = 1000000.0 / rFsr;
  }

  // Display values in a tabulated format
  Serial.print(adcVal);
  Serial.print("\t");
  Serial.print(vOut, 3);      // 3 decimal places
  Serial.print("\t\t");
  
  if (rFsr == 0) Serial.print("INF");
  else Serial.print(rFsr, 1);
  
  Serial.print("\t\t");
  Serial.println(conductance, 4);

  delay(500); // Slow down for easier reading during experiment
}
