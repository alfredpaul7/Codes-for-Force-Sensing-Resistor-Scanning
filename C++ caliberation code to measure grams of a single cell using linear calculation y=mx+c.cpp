//caliberation code to measure grams of a single cell using linear calculation y=mx+c
#include <Arduino.h>
// --- Pin Definitions (Hardware) ---
const int ser    = 11; 
const int srclk  = 13; 
const int rclk   = 10; 
const int sPins[] = {2, 3, 4, 5}; // MUX control pins
const int rowB    = A1;           // Analog reading pin
// --- Electrical Constants ---
const float VCC      = 3.3;       // System Voltage
const float R_FIXED  = 10000.0;   // 10k Ohm fixed resistor
const float ADC_RES  = 4096.0;    // 12-bit ADC Resolution
// --- Your Derived Calibration Constants ---
// Calculated from your specific sensor readings
const float SLOPE    = 11.826;    // Derived from your CSV data
const float OFFSET   = 0.171;     // Derived from your CSV data
void setup() {
  Serial.begin(115200);
  // Initialize Pins
  pinMode(ser, OUTPUT);
  pinMode(srclk, OUTPUT);
  pinMode(rclk, OUTPUT);
  for (int i = 0; i < 4; i++) {
    pinMode(sPins[i], OUTPUT);
  }
  // --- 1. ACTIVATE SHIFT REGISTER (Pin Qa at position 32) ---
  digitalWrite(rclk, LOW);
  for (int i = 1; i <= 32; i++) {
    digitalWrite(srclk, LOW);
    // Send HIGH bit to the last position as per your requirement
    digitalWrite(ser, (i == 32) ? HIGH : LOW); 
    digitalWrite(srclk, HIGH);
  }
  digitalWrite(rclk, HIGH);
  // --- 2. SET MUX TO CHANNEL 15 (S0-S3 all HIGH) ---
  for (int i = 0; i < 4; i++) {
    digitalWrite(sPins[i], HIGH);
  }
  Serial.println("RoxiFSR System Initialized");
  Serial.println("Reading Weight in Grams...");
  Serial.println("--------------------------------");
}
void loop() {
  // Small delay for analog signal stabilization
  delayMicroseconds(30); 
  // 1. Read Raw 12-bit ADC value
  int adcVal = analogRead(rowB);
  // 2. Convert ADC to Voltage (V_out)
  float vOut = (adcVal * VCC) / ADC_RES;
  float massGrams = 0;
  // 3. Perform Calculations if weight is detected
  // A threshold of 15-20 helps ignore electrical noise at 0g
  if (adcVal > 15) {
    // Calculate Resistance (R_fsr)
    float rFsr = ((VCC - vOut) * R_FIXED) / vOut;
    // Calculate Conductance (G) in microSiemens
    float conductance = 1000000.0 / rFsr;
    // 4. Apply Linear Calibration Formula: Mass = (Slope * G) + Offset
    massGrams = (SLOPE * conductance) + OFFSET;
    // Safety check to ensure mass isn't negative due to tiny offset noise
    if (massGrams < 0) massGrams = 0;
  }
  // 5. Output to Serial Monitor
  Serial.print("ADC: ");
  Serial.print(adcVal);
  Serial.print(" | Weight: ");
  Serial.print(massGrams, 1); // Display with 1 decimal place
  Serial.println(" g");
  delay(500); // 2 readings per second
}
