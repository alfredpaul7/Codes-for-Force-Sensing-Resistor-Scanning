#include <Arduino.h>
#include <math.h> 

// --- Pin Definitions (Hardware) ---
const int ser = 11; const int srclk = 13; const int rclk = 10;
const int sPins[] = {2, 3, 4, 5};
const int rowB = A1;

// --- Electrical Constants ---
const float VCC = 3.3; 
const float R_FIXED = 10000.0; 
const float ADC_RES = 4096.0;

// --- 1. Linear Method Constants --- y=mx+c
const float LIN_M = 11.8258;
const float LIN_C = 0.1711;

// --- 2. Power Law Method Constants ---y=ax^b +c
const float PWR_A = 15.2683;
const float PWR_B = 0.9093;

// --- 3. 4th Order Polynomial Constants ---
// Formula: y = Ax^4 + Bx^3 + Cx^2 + Dx + E
const float POLY_A = 0.000527; 
const float POLY_B = -0.03139;
const float POLY_C = 0.44914;
const float POLY_D = 9.48081;
const float POLY_E = 7.57228;

void setup() {
  Serial.begin(115200);
  pinMode(ser, OUTPUT); pinMode(srclk, OUTPUT); pinMode(rclk, OUTPUT);
  for (int i = 0; i < 4; i++) pinMode(sPins[i], OUTPUT);

  // Initialize hardware to read the specific sensor spot
  digitalWrite(rclk, LOW);
  for (int i = 1; i <= 32; i++) {
    digitalWrite(srclk, LOW);
    digitalWrite(ser, (i == 32) ? HIGH : LOW); 
    digitalWrite(srclk, HIGH);
  }
  digitalWrite(rclk, HIGH);
  for (int i = 0; i < 4; i++) digitalWrite(sPins[i], HIGH);

  Serial.println("ADC | Linear (g) | Power (g) | Poly 4th (g)");
  Serial.println("------------------------------------------");
}

void loop() {
  delayMicroseconds(30); 
  int adcVal = analogRead(rowB);
  float vOut = (adcVal * VCC) / ADC_RES;

  float mLin = 0, mPwr = 0, mPoly = 0;

  if (adcVal > 15) {
    float rFsr = ((VCC - vOut) * R_FIXED) / vOut;
    float G = 1000000.0 / rFsr; // Conductance in uS

    // --- METHOD 1: LINEAR ---
    mLin = (LIN_M * G) + LIN_C;

    // --- METHOD 2: POWER LAW ---
    mPwr = PWR_A * pow(G, PWR_B);

    // --- METHOD 3: 4th ORDER POLYNOMIAL ---
    // Efficiently calculated using Horner's Method
    mPoly = POLY_E + G * (POLY_D + G * (POLY_C + G * (POLY_B + G * POLY_A)));

    // Safety: No negative weights
    if (mLin < 0) mLin = 0; if (mPwr < 0) mPwr = 0; if (mPoly < 0) mPoly = 0;
  }

  // Displaying all three for comparison
  Serial.print(" ADC: ");Serial.print(adcVal);
  Serial.print(" | L: "); Serial.print(mLin, 1); 
  Serial.print(" | P: "); Serial.print(mPwr, 1); 
  Serial.print(" | Poly: "); Serial.println(mPoly, 1); 

  delay(500);
}
