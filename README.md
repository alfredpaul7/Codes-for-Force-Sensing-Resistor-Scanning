# Documentation for Interfacing and data Acquisation of  Force sensing resistor.

***You can also view this documentation on the web or on Notion. The links are provided below. The project report can be accessed using the link given.***

**WEB Link :** [https://laced-cap-19e.notion.site/Documentation-for-Interfacing-and-data-Acquisation-of-Force-sensing-resistor-2f835f708a9380aaaa4dd4aeb9c6d309?source=copy_link](https://www.notion.so/Documentation-for-Interfacing-and-data-Acquisation-of-Force-sensing-resistor-2f835f708a9380aaaa4dd4aeb9c6d309?pvs=21)

**Notion link:** [https://www.notion.so/Documentation-for-Interfacing-and-data-Acquisation-of-Force-sensing-resistor-2f835f708a9380aaaa4dd4aeb9c6d309?source=copy_link](https://www.notion.so/Documentation-for-Interfacing-and-data-Acquisation-of-Force-sensing-resistor-2f835f708a9380aaaa4dd4aeb9c6d309?pvs=21)

**Report link :** https://drive.google.com/drive/folders/1ozw_Ty0pGpx6KO5M5wNeE8qQ3ilpqTL_?usp=sharing

**Github repository:** https://github.com/alfredpaul7/Codes-for-Force-Sensing-Resistor-Scanning

## Introduction

This Documentation explains the complete process of interfacing, scanning, visualizing, and calibrating **Force Sensing Resistor (FSR) matrix sensors** using low-cost embedded hardware. The work covers sensor configurations starting from a **single 32×32 FSR matrix** and scaling up to **32×64 and 64×64 FSR matrices** using serial and SPI communication.

## General Explanation of Force Sensing Resistor (FSR)

A Force Sensing Resistor (FSR) is a piezoresistive sensor whose electrical resistance decreases when force or pressure is applied. FSRs are thin, flexible, and lightweight, making them suitable for large-area sensing applications.

When multiple FSR elements are arranged in rows and columns, they form an **FSR matrix**. Each sensing cell (sensel) is located at the intersection of a row and a column. Since FSRs are passive and nonlinear devices, matrix-based systems require proper scanning techniques using multiplexers and shift registers to read each cell accurately.

**Key points to mention:**

- Resistance ↓ with force ↑
- Conductance (1/R) is approximately linear with force
- Best suited for **relative pressure mapping**, not precision load cells

## Components Used

### Arduino Nano ESP32

The Arduino Nano ESP32 is a compact microcontroller board based on the ESP32-S3. It operates at 3.3 V logic level and includes a 12-bit ADC (0–4095), SPI support, and high processing speed. It is well suited for real-time data acquisition and large sensor matrix scanning.

### Breadboard

A breadboard is used for prototyping and testing the interfacing circuit without the need for a custom PCB.

### Force Sensing Resistor Matrix (32×32)

The 32×32 FSR matrix contains 1024 individual sensing cells. Each cell behaves as a variable resistor whose resistance decreases with applied pressure. In our system we use Roxifsr RX3232l FSR

[3232柔性分布式压力测试系统-常州柔希电子科技有限公司](https://www.roxifsr.com/productinfo/3160053.html)

### Shift Registers (SN74HC595)

The SN74HC595 is an 8-bit serial-in, parallel-out shift register. It allows multiple output lines to be controlled using only a few microcontroller pins. Cascading multiple shift registers enables control of a large number of FSR columns. For 32x32 FSR we use 4 shift registers , for 32x64 FSR we use 8 shift registers and for 64x64 FSR matrix we use 16 shift registers

### Analog Multiplexers (CD74HC4067)

The CD74HC4067 is a 16-channel analog multiplexer. It is used to route multiple row signals to a single ADC pin, reducing wiring complexity and GPIO usage. For 32x32 FSR we use 2 Multiplexers , for 32x64 FSR we use 4 Multiplexer and for 64x64 FSR matrix we use 8 mutiplexer

### 10 kΩ Resistors

10 kΩ resistors are used as fixed reference resistors in voltage divider circuits with the FSR cells to convert resistance changes into measurable voltages. It is connected between the SIG pin of each mux to the Analog pin of Arduino (A0-A7)

### 0.1 µF Capacitors

0.1 µF capacitors are used as decoupling capacitors to reduce noise and stabilize the power supply during fast switching operations. The ceramic capacitors are connected between the VCC and GND of each shift registers

**JUMPER WIRES**

The jumper wires are used to connect the FSR with the components such has shift registers and multiplexers. Each FSR needs 64jumper wires.

## Interfacing 1 × 32×32 FSR Using Serial Communication

### System Overview

In this configuration, FSR matrix columns are activated sequentially using shift registers, and row signals are read using analog multiplexers. The Arduino reads the analog voltage corresponding to each sensing cell and transmits the data using serial communication.

### Circuit Connection Table

| COMPONENT | PIN | CONNECTED TO | DESCRIPTION |
| --- | --- | --- | --- |
| CD74HC4067 (Mux1) | SIG | Arduino A0 via 10 kΩ resistor | Analog signal output for FSR rows 1–16 |
| CD74HC4067 (Mux2) | SIG | Arduino A1 via 10 kΩ resistor | Analog signal output for FSR rows 17–32 |
| CD74HC4067 (Both) | EN | GND | Enables multiplexer (active low) |
| CD74HC4067 (Both) | S0 | Arduino D2 | Channel select bit 0 |
| CD74HC4067 (Both) | S1 | Arduino D3 | Channel select bit 1 |
| CD74HC4067 (Both) | S2 | Arduino D4 | Channel select bit 2 |
| CD74HC4067 (Both) | S3 | Arduino D5 | Channel select bit 3 |
| CD74HC4067 (Mux1) | C0–C15 | FSR Rows R1–R16 | Row signal inputs |
| CD74HC4067 (Mux2) | C0–C15 | FSR Rows R17–R32 | Row signal inputs |
| SN74HC595 Shift Register | SER | Arduino D11 | Serial data input |
| SN74HC595N (SR1–SR4) | QH′ | SER of next shift register | Cascading shift registers |
| SN74HC595N (All) | RCLK | Arduino D10 | Latch clock |
| SN74HC595N (All) | SRCLK | Arduino D13 | Shift clock |
| SN74HC595N (All) | RCLR | VCC | Reset disabled (active low) |
| SN74HC595N (All) | Q0–Q7 | FSR Column Channels (32 columns) | Column excitation outputs |
| All Shift Registers & Mux | VCC | Arduino VCC (3.3 V) | Power supply |
| All Shift Registers & Mux | GND | Arduino GND | Common ground |

### C++ Code (Arduino)

The Arduino code performs the following tasks:

- Initializes GPIO, ADC, and serial communication
- Activates one column at a time using shift registers
- Selects rows using multiplexers
- Reads ADC values
- Sends 1024 values per frame over serial communicatio

```cpp
//C++ code for 32x32 with serial communication
#include<Arduino.h>
// --- 74HC595 Pin Definitions (Drive 32 Columns) ---
const int ser    = 13; //595 Data Pin (DS, Pin 14)
const int srclk  = 12; // 595 Clock Pedal (SHCP, Pin 11)
const int rclk  = 11; // 595 Latch pin (STCP, Pin 12)
// --- CD74HC4067 Pin Definitions (Read line 32) ---
const int sPins[]  = {2, 3, 4, 5};// Address lines S0, S1, S2, S3
const int rowA     = A0;           // MUX 1 output (Row 1-16)
const int rowB     = A1;           // MUX 2 output (Row 17-32)
void setActiveColumn(int col);
void clearAllColumns();
void clearAllColumns();
void setup() {
  // Use a high baud rate to ensure fast transmission of 1,024 points.
  Serial.begin(115200);
  // Initialize 595 control pin
  pinMode(ser, OUTPUT);//ser sending bit value to the register
  pinMode(srclk, OUTPUT);//srclk  sending the data insidethe resister
  pinMode(rclk, OUTPUT);//rclk updates output all at once  latch 
  // Initialize MUX address control pins
  for (int i = 0; i < 4; i++) {
    pinMode(sPins[i], OUTPUT);
  }
  // Initial state: clear all columns
  clearAllColumns();  
}
void loop() {
  // Iterate through 32 columns (1 ~ 32)
  for (int col = 1; col <= 32; col++) {
    // 1. Power on the current column (3.3V)
    setActiveColumn(col);
    // 2. Switch MUX to read the 32 rows in the current column
    // Because there are two MUXs connected to A0 and A1 respectively, it only takes 16 loops to read all 32 rows.
    for (int ch = 0; ch < 16; ch++) {
      // Set the 4-bit address of the MUX (S0-S3))
      digitalWrite(sPins[0], (ch & 0x01));
      digitalWrite(sPins[1], (ch >> 1) & 0x01);
      digitalWrite(sPins[2], (ch >> 2) & 0x01);
      digitalWrite(sPins[3], (ch >> 3) & 0x01);
      // Provides an extremely short settling time for the analog signal (stabilizing voltage divider circuit)
      delayMicroseconds(30);
      // Read A0 (Row 1-16) and A1 (Row 17-32)
      int val1 = analogRead(rowA); 
      int val2 = analogRead(rowB);
      // --- Output data for Python to parse ---
      // Output format: val,val,val...
      Serial.print(val1);
      Serial.print(",");
      Serial.print(val2);
      // Add a comma if it's not the last point in the entire image.
      if (ch < 15 || col < 32) {
        Serial.print(",");
      }
    }
  }
 // Once all 1,024 dots have been transmitted, a newline character is sent.
  Serial.println();
}
// --- Core function: Precisely selects a specific column from 32 bits ---
void setActiveColumn(int col) { 
  // Preparing for an update, first lower the latch
  digitalWrite(rclk, LOW);
  // 4 595 cascaded chips require a total of 32 bits to be transferred
  // Here, the transfer starts from the 32nd bit and moves to the 1st bit (FIFO, pushed to the last chip)
  for (int i = 32; i >= 1; i--) {
    digitalWrite(srclk, LOW);
    // Only the target is listed as HIGH, the rest as LOW.
    if (i == col) {
      digitalWrite(ser, HIGH);
    } else {
      digitalWrite(ser, LOW);
    }
    digitalWrite(srclk, HIGH);// Input bit
  }
  // Transfer complete, pull the latch high to update the output status
  digitalWrite(rclk, HIGH);
}
// --- Helper function: Clear the voltage of all columns ---
void clearAllColumns() {
  digitalWrite(rclk, LOW);
  for (int i = 0; i < 32; i++) {
    digitalWrite(srclk, LOW);
    digitalWrite(ser, LOW);
    digitalWrite(srclk, HIGH);
  }
  digitalWrite(rclk, HIGH);
}
```

### Python Heatmap Code

The Python program:

- Reads serial data from the Arduino
- Reshapes the data into a 32×32 matrix
- Applies basic thresholding to reduce noise
- Displays a real-time heatmap using Matplotlib

```cpp
#Python heatmap code for 32x32 with serial communication 
import serial
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# --- Settings Area ---
COM_PORT = 'COM6'
BAUD_RATE = 115200
ROWS = 32
COLS = 32

try:
    ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=0.1)
    ser.flushInput()
    print(f"Successfully connected to {COM_PORT}")
except Exception as e:
    print(f"Error: {e}")
    exit()

# Setup Figure and Axis
fig, ax = plt.subplots(figsize=(10, 8))
data_matrix = np.zeros((ROWS, COLS))

# Render the heatmap
im = ax.imshow(data_matrix, cmap='jet', interpolation='gaussian', vmin=0, vmax=1500)

# --- COLORBAR SETTINGS ---
# Create the colorbar and remove the numeric labels (0-1400)
cbar = plt.colorbar(im)
cbar.set_ticks([]) 

# --- ADDING ROW AND COLUMN NUMBERING ---
ax.set_xticks(np.arange(COLS))
ax.set_yticks(np.arange(ROWS))

# Label every single sensel (1-indexed)
ax.set_xticklabels([str(i+1) for i in range(COLS)], fontsize=8)
ax.set_yticklabels([str(i+1) for i in range(ROWS)], fontsize=8)

# Move X-axis labels to the top
ax.xaxis.tick_top()

# Add Grid to make it look like a matrix
ax.set_xticks(np.arange(-.5, COLS, 1), minor=True)
ax.set_yticks(np.arange(-.5, ROWS, 1), minor=True)
ax.grid(which='minor', color='white', linestyle='-', linewidth=0.5)

def update(frame):
    if ser.in_waiting > 0:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not line: return [im]
            
            raw_data = [int(x) for x in line.split(',') if x.strip()]
            
            if len(raw_data) == ROWS * COLS:
                # 1. Reshape
                new_matrix = np.array(raw_data).reshape((ROWS, COLS))
                
                # 2. Fix Horizontal/Vertical Swap
                new_matrix = new_matrix.T
                
                # 3. FIX INVERSION
                # Note: I removed the quadruple flips (ud/lr/ud/lr) as they cancel each other out.
                # If your image appears upside down or mirrored, uncomment only ONE of these:
                # new_matrix = np.flipud(new_matrix) # Use this if image is upside down
                # new_matrix = np.fliplr(new_matrix) # Use this if image is mirrored
                
                im.set_data(new_matrix)
            else:
                pass
        except Exception as e:
            print(f"Parsing error: {e}")
    return [im]

ani = FuncAnimation(fig, update, interval=20, blit=True, cache_frame_data=False)
plt.tight_layout()
plt.show()
ser.close()
```

## Interfacing 1 × 32×32 FSR Using SPI Communication

### Need for SPI Communication

During the initial implementation with a 32×32 FSR matrix, serial communication at 115200 baud was sufficient to transmit 1024 sensor values per scan cycle. However, when the system was expanded to a 32×64 FSR matrix, the output data doubled to 2048 values per frame. At this data rate, the Arduino Nano ESP32 was unable to reliably transmit and process all values using standard serial communication. This resulted in frame drops,and unstable [visualization.To](http://visualization.to/) overcome this limitation a faster communication method was required, leading to the adoption of the SPI communication protocol.

### Circuit Connection Table

Same as the serial communcation

### C++ Code (Arduino with SPI)

The SPI-based Arduino code:

- Initializes the SPI interface
- Sends column selection data as bitmasks using SPI.transfer()
- Reduces scanning time and improves synchronizatio

```cpp
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
```

### Python Heatmap Code

The Python code remains similar to the serial version but supports higher and more stable data rates.

```cpp
#Python heatmap code for 32x32 with SPI protocol 
import serial
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# --- Settings ---
COM_PORT = 'COM6' 
BAUD_RATE = 921600  # Matched to Arduino code
ROWS = 32
COLS = 32

try:
    ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=0.1)
    ser.flushInput()
    print(f"Connected to {COM_PORT} at {BAUD_RATE}")
except Exception as e:
    print(f"Error: {e}")
    exit()

fig, ax = plt.subplots(figsize=(10, 8))
data_matrix = np.zeros((ROWS, COLS))
im = ax.imshow(data_matrix, cmap='jet', interpolation='gaussian', vmin=0, vmax=1023)
plt.colorbar(im)

# Formatting
ax.set_xticks(np.arange(COLS))
ax.set_yticks(np.arange(ROWS))
ax.set_xticklabels([str(i+1) for i in range(COLS)], fontsize=8)
ax.set_yticklabels([str(i+1) for i in range(ROWS)], fontsize=8)
ax.xaxis.tick_top()

def update(frame):
    if ser.in_waiting > 0:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not line: return [im]
            
            raw_data = [int(x) for x in line.split(',') if x.strip()]
            
            if len(raw_data) == ROWS * COLS:
                # Reshape and Transpose to align with Column-scanning logic
                new_matrix = np.array(raw_data).reshape((COLS, ROWS)).T 
                
                # Correct orientation: adjust these only once to match physical sensor
                # new_matrix = np.flipud(new_matrix) 
                
                im.set_data(new_matrix)
        except Exception as e:
            pass 
    return [im]

ani = FuncAnimation(fig, update, interval=1, blit=True, cache_frame_data=False)
plt.tight_layout()
plt.show()
ser.close()
```

## Interfacing 2 pcs of  32×32 FSR to Form a 32×64 Matrix (SPI)

### System Overview

Two 32×32 FSR matrices are placed side by side to form a 32×64 sensing surface. Additional shift registers and multiplexers are used to handle the increased number of columns and rows.

### Circuit Connection Table

| COMPONENT | PIN | CONNECTED TO | DESCRIPTION |
| --- | --- | --- | --- |
| CD74HC4067 (Mux1) | SIG | Arduino A0 via 10 kΩ resistor | Analog signal output for FSR 1 rows 1–16 |
| CD74HC4067 (Mux2) | SIG | Arduino A1 via 10 kΩ resistor | Analog signal output for FSR 1 rows 17–32 |
| CD74HC4067 (Mux3) | SIG | Arduino A2 via 10 kΩ resistor | Analog signal output for FSR 2 rows 1–16 |
| CD74HC4067 (Mux4) | SIG | Arduino A3 via 10 kΩ resistor | Analog signal output for FSR 2 rows 17–32 |
| CD74HC4067 (Both) | EN | GND | Enables multiplexer (active low) |
| CD74HC4067 (Both) | S0 | Arduino D2 | Channel select bit 0 |
| CD74HC4067 (Both) | S1 | Arduino D3 | Channel select bit 1 |
| CD74HC4067 (Both) | S2 | Arduino D4 | Channel select bit 2 |
| CD74HC4067 (Both) | S3 | Arduino D5 | Channel select bit 3 |
| CD74HC4067 (Mux1) | C0–C15 | FSR1 Rows R1–R16 | Row signal inputs |
| CD74HC4067 (Mux2) | C0–C15 | FSR1 Rows R17–R32 | Row signal inputs |
| CD74HC4067 (Mux3) | C0–C15 | FSR2 Rows R1–R16 | Row signal inputs |
| CD74HC4067 (Mux4) | C0–C15 | FSR2 Rows R17–R32 | Row signal inputs |
| SN74HC595 Shift Register | SER | Arduino D11 | Serial data input |
| SN74HC595N (SR1–SR4) | QH′ | SER of next shift register | Cascading shift registers |
| SN74HC595N (All) | RCLK | Arduino D10 | Latch clock |
| SN74HC595N (All) | SRCLK | Arduino D13 | Shift clock |
| SN74HC595N (All) | RCLR | VCC | Reset disabled (active low) |
| SN74HC595N (All) | Q0–Q7 | FSR Column Channels | Column excitation outputs |
| All Shift Registers & Mux | VCC | Arduino VCC (3.3 V) | Power supply |
| All Shift Registers & Mux | GND | Arduino GND | Common ground |

### C++ Code

The Arduino firmware:

- Generates a 64-bit column selection mask
- Scans both FSR matrices sequentially
- Sends 2048 ADC values per frame

```cpp
//C++ code for 32x64 with SPI protocol
#include <Arduino.h>
#include <SPI.h>

// --- SPI Pin Definitions ---
/*RCLK D10
SRCLK  D13
SER D11*/
const int slaveSelectPin = 10; 
// --- CD74HC4067 Pin Definitions ---
const int sPins[] = {2, 3, 4, 5};
// --- Analog Rows ---
// Sensor 1 (Left)
const int rowA = A0; 
const int rowB = A1; 
// Sensor 2 (Right)
const int rowC = A2; 
const int rowD = A3; 
void setActiveColumn(int col);
void clearAllColumns();
void setMuxAddress(int ch);
void setup() {
  Serial.begin(921600);
  pinMode(slaveSelectPin, OUTPUT);
  digitalWrite(slaveSelectPin, HIGH);
  SPI.begin();
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  for (int i = 0; i < 4; i++) {
    pinMode(sPins[i], OUTPUT);
  }
  clearAllColumns();
}
void loop() {
  // --- PART 1: READ SENSOR 1 (Left) ---
  for (int col = 0; col < 32; col++) {
    setActiveColumn(col);
    for (int ch = 0; ch < 16; ch++) {
      setMuxAddress(ch);
      delayMicroseconds(20); 
      Serial.print(analogRead(rowA));
      Serial.print(",");
      Serial.print(analogRead(rowB));
      Serial.print(","); // Always trailing comma here
    }
  }
  // --- PART 2: READ SENSOR 2 (Right) ---
  for (int col = 0; col < 32; col++) {
    setActiveColumn(col);
    for (int ch = 0; ch < 16; ch++) {
      setMuxAddress(ch);
      delayMicroseconds(20); 
      Serial.print(analogRead(rowC));
      Serial.print(",");
      Serial.print(analogRead(rowD));
      // Only add comma if it's NOT the very last value of the whole 2048 set
      if (ch < 15 || col < 31) {
        Serial.print(",");
      }
    }
  }
  Serial.println(); // Single Newline to end the 2048-value frame
}
void setMuxAddress(int ch) {
  digitalWrite(sPins[0], (ch & 0x01));
  digitalWrite(sPins[1], (ch >> 1) & 0x01);
  digitalWrite(sPins[2], (ch >> 2) & 0x01);
  digitalWrite(sPins[3], (ch >> 3) & 0x01);
}
void setActiveColumn(int col) {
  uint32_t bitmask = 0x00000001UL << col;
  digitalWrite(slaveSelectPin, LOW);
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
```

### Python Heatmap Code

The Python program:

- Splits incoming data into two 32×32 blocks
- Transposes the data for correct orientation
- Merges both matrices into a single 32×64 heatmap

```cpp
#Python heatmap code for 32x64 with SPI protocol
import serial
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
# --- Settings ---
COM_PORT = 'COM6'
BAUD_RATE = 921600
ROWS, COLS = 32, 64  # Updated for your 32x64 display
THRESHOLD = 40       
try:
    ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=0.1)
    ser.flushInput()
    print(f"Connected to {COM_PORT}")
except Exception as e:
    print(f"Error: {e}")
    exit()
fig, ax = plt.subplots(figsize=(10, 8)) # Wider figure for 32x64
data_matrix = np.zeros((ROWS, COLS))
# Using 'Gaussian' interpolation for that smooth heatmap look
im = ax.imshow(data_matrix, cmap='jet', interpolation='gaussian', vmin=0, vmax=1500)
plt.colorbar(im)
# Formatting labels
ax.set_xticks(np.arange(COLS)) # Labels every 4 units for readability
ax.set_yticks(np.arange(ROWS))
ax.set_xticklabels([str(i+1) for i in range(COLS)], fontsize=6)
ax.set_yticklabels([str(i+1) for i in range(ROWS)], fontsize=6)
ax.xaxis.tick_top()
def update(frame):
    if ser.in_waiting > 0:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not line: return [im]  
            # Convert the long string of 2048 values into a list of integers
            raw_data = [int(x) for x in line.split(',') if x.strip()]
            if len(raw_data) == 2048:
                # Process Left Sensor (Values 0-1023)
                tile_left = np.array(raw_data[:1024]).reshape((32, 32)).T

                # Process Right Sensor (Values 1024-2047)
                tile_right = np.array(raw_data[1024:]).reshape((32, 32)).T
            
                # Combine into the 32x64 canvas
                full_canvas = np.zeros((32, 64))
                full_canvas[:, :32] = tile_left
                full_canvas[:, 32:] = tile_right
                full_canvas[full_canvas < THRESHOLD] = 0
                im.set_data(full_canvas)
              
            else:
                # This helps you debug if you aren't sending enough data
                print(f"Data received but length is {len(raw_data)} instead of 2048")
              
        except Exception as e:
            print(f"Parsing error: {e}")
    return [im]
ani = FuncAnimation(fig, update, interval=1, blit=True, cache_frame_data=False)
plt.tight_layout()
plt.show()
ser.close()
```

## Interfacing 4 pcs of 32×32 FSR to Form a 64×64 Matrix (SPI) but working needs to be checked

### System Overview

Four 32×32 FSR matrices are combined to form a 64×64 sensing surface. The same scanning principles are extended using additional shift registers and multiplexers.

### Circuit Connection Table

### Multiplexer Connections (Row Scanning)

| COMPONENT | PIN | CONNECTED TO | DESCRIPTION |
| --- | --- | --- | --- |
| CD74HC4067 (Mux1) | SIG | Arduino A0 via 10 kΩ resistor | Analog output for FSR-1 rows 1–16 |
| CD74HC4067 (Mux2) | SIG | Arduino A1 via 10 kΩ resistor | Analog output for FSR-1 rows 17–32 |
| CD74HC4067 (Mux3) | SIG | Arduino A2 via 10 kΩ resistor | Analog output for FSR-2 rows 1–16 |
| CD74HC4067 (Mux4) | SIG | Arduino A3 via 10 kΩ resistor | Analog output for FSR-2 rows 17–32 |
| CD74HC4067 (Mux5) | SIG | Arduino A4 via 10 kΩ resistor | Analog output for FSR-3 rows 1–16 |
| CD74HC4067 (Mux6) | SIG | Arduino A5 via 10 kΩ resistor | Analog output for FSR-3 rows 17–32 |
| CD74HC4067 (Mux7) | SIG | Arduino A6 via 10 kΩ resistor | Analog output for FSR-4 rows 1–16 |
| CD74HC4067 (Mux8) | SIG | Arduino A7 via 10 kΩ resistor | Analog output for FSR-4 rows 17–32 |
| CD74HC4067 (All) | EN | GND | Enables multiplexers (active low) |
| CD74HC4067 (All) | S0 | Arduino D2 | Channel select bit 0 |
| CD74HC4067 (All) | S1 | Arduino D3 | Channel select bit 1 |
| CD74HC4067 (All) | S2 | Arduino D4 | Channel select bit 2 |
| CD74HC4067 (All) | S3 | Arduino D5 | Channel select bit 3 |
| CD74HC4067 (Mux1) | C0–C15 | FSR-1 Rows R1–R16 | Row signal inputs |
| CD74HC4067 (Mux2) | C0–C15 | FSR-1 Rows R17–R32 | Row signal inputs |
| CD74HC4067 (Mux3) | C0–C15 | FSR-2 Rows R1–R16 | Row signal inputs |
| CD74HC4067 (Mux4) | C0–C15 | FSR-2 Rows R17–R32 | Row signal inputs |
| CD74HC4067 (Mux5) | C0–C15 | FSR-3 Rows R1–R16 | Row signal inputs |
| CD74HC4067 (Mux6) | C0–C15 | FSR-3 Rows R17–R32 | Row signal inputs |
| CD74HC4067 (Mux7) | C0–C15 | FSR-4 Rows R1–R16 | Row signal inputs |
| CD74HC4067 (Mux8) | C0–C15 | FSR-4 Rows R17–R32 | Row signal inputs |

### Shift Register Connections (Column Scanning)

| COMPONENT | PIN | CONNECTED TO | DESCRIPTION |
| --- | --- | --- | --- |
| SN74HC595 (SR1) | SER | Arduino D11 | Serial data input |
| SN74HC595 (SR1–SR16) | QH′ | SER of next shift register | Cascading shift registers |
| SN74HC595 (All) | RCLK | Arduino D10 | Latch clock |
| SN74HC595 (All) | SRCLK | Arduino D13 | Shift clock |
| SN74HC595 (All) | RCLR | VCC | Reset disabled (active low) |
| SN74HC595 (All) | Q0–Q7 | FSR Column Channels (64 columns) | Column excitation outputs |

### Power and Ground Connections

| COMPONENT | PIN | CONNECTED TO | DESCRIPTION |
| --- | --- | --- | --- |
| All Shift Registers & Mux | VCC | Arduino VCC (3.3 V) | Power supply |
| All Shift Registers & Mux | GND | Arduino GND | Common ground |

### C++ Code

The firmware:

- Scans all 4096 sensing cells
- Maintains stable SPI-based column control
- Streams data efficiently for visualization

```
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
```

### Python Heatmap Code

The Python code reconstructs a 64×64 matrix and displays a real-time pressure heatmap

```cpp
import serial
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# --- Settings ---
COM_PORT = 'COM6'  # Change to your actual port
BAUD_RATE = 921600
ROWS, COLS = 64, 64
THRESHOLD = 40 
EXPECTED_SAMPLES = 4096 

try:
    ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=0.1)
    ser.flushInput()
    print(f"Connected to {COM_PORT}")
except Exception as e:
    print(f"Error: {e}")
    exit()

fig, ax = plt.subplots(figsize=(10, 10))
data_matrix = np.zeros((ROWS, COLS))

# 'Gaussian' interpolation creates a smooth heat look
im = ax.imshow(data_matrix, cmap='jet', interpolation='gaussian', vmin=0, vmax=1024)
plt.colorbar(im)

ax.xaxis.tick_top()
ax.set_title("64x64 FSR Matrix Real-Time Heatmap")

def update(frame):
    if ser.in_waiting > 0:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not line: return [im]

            raw_data = [int(x) for x in line.split(',') if x.strip()]
            
            if len(raw_data) == EXPECTED_SAMPLES:
                # 1. Convert to numpy array
                arr = np.array(raw_data)
                
                # 2. Reshape to match the Arduino loop: (Columns, MuxChannels, Pins)
                # We have 64 columns, 16 Mux channels, and 8 pins read per channel
                temp = arr.reshape((64, 16, 8))
                
                full_canvas = np.zeros((64, 64))
                
                # 3. Map the data to the correct coordinates
                for col in range(64):
                    for ch in range(16):
                        for p in range(8):
                            # Determine Row: 
                            # Pins 0-3 (rowsTop) go to rows (ch, ch+16, etc.)
                            # This mapping depends on how you physically wired the sensors
                            row_offset = 32 if p >= 4 else 0
                            actual_row = ch + row_offset
                            
                            # You may need to adjust 'actual_row' or 'col' based on
                            # if the sensors are oriented horizontally or vertically.
                            full_canvas[actual_row, col] = temp[col, ch, p]

                # Apply noise threshold
                full_canvas[full_canvas < THRESHOLD] = 0
             
                # Update the display
                im.set_data(full_canvas)
                       else:
                print(f"Packet Error: Received {len(raw_data)} samples.")   
        except Exception as e:
            print(f"Data Error: {e}")
    return [im]
ani = FuncAnimation(fig, update, interval=1, blit=True, cache_frame_data=False)
plt.tight_layout()
plt.show()

ser.close()
```

# A Proposed Calibration Methodology for the Conversion of ADC Values to Mass (in Grams)

Calibration is an essential step in **Force Sensing Resistor (FSR)–based measurement systems**, as it establishes a meaningful relationship between the raw electrical output of the sensor and the actual applied force or weight. In FSR systems, calibration involves converting **ADC readings obtained from the microcontroller into physical units such as grams**. This is necessary because FSRs exhibit **nonlinear behavior** and do not provide direct force measurements.

Proper calibration improves **measurement accuracy and repeatability**, which is especially important when working with **large FSR matrices**, where a uniform response across all sensing elements is required.

## ADC Data Acquisition for Calibration

ADC data acquisition for calibration is performed by applying **known reference weights** to the sensor and recording the corresponding ADC values. To reduce noise and improve stability, multiple ADC samples are collected for each applied weight and then averaged.

The averaged ADC values are processed as follows:

- ADC values are converted into output voltage V_out using the ADC resolution and reference voltage.
- The FSR resistance R_fsr is calculated using the voltage divider equation.
- Conductance G=1/R_fsr is computed to obtain a more linear relationship with applied force.

Based on this processed data, a **linear calibration model** is evaluated to map conductance to applied weight. These calibration models help compensate for **bias, scaling variations, hysteresis, and drift**, enabling stable and reliable force measurement in practical applications.

## Measurement of ADC Values Using Known Reference Weights

ADC readings are obtained by placing **known reference weights (20 g, 50 g, 100 g, 200 g, and 500 g)** on a **single sensing cell** of the FSR matrix. Each weight is applied at the same location to maintain consistent loading conditions.

For every applied load:

- Multiple ADC samples are recorded
- The samples are averaged to minimize noise and improve measurement stability

The averaged ADC values are then used to compute:

- Output voltage V_out
- FSR resistance R_fsr
- Conductance G

These parameters serve as inputs to the calibration equations, which are finally used to convert raw ADC values into **physical units such as grams**.

## Alternative Calibration Using a Material Testing Machine

An alternative approach for acquiring ADC values involves applying **controlled loads using a material testing machine**. This method provides higher accuracy and repeatability compared to discrete weights and enables precise characterization of the FSR response under well-defined loading conditions.

A similar calibration approach using a material testing setup is discussed in the research paper:

**“Evaluation of Force Sensing Resistors for the Measurement of Interface Pressures in Lower Limb Prosthetics.”**

---

## Calculation of V_out , R_fsr, and Conductance

To establish a reliable calibration model, ADC readings were collected by applying known discrete weights to a single sensing cell of the FSR matrix. For each reference mass (20 g, 50 g, 100 g, 200 g, and 500 g), approximately **60 ADC samples** were recorded under identical loading conditions.

These samples were averaged to reduce noise and improve measurement stability. The averaged ADC values form the basis for converting raw sensor output into electrical parameters used for calibration.

## Step 1: ADC to Voltage Conversion ( V_out)

The Arduino Nano ESP32 uses a **12-bit ADC**, producing values in the range **0 to 4095**. Each averaged ADC value is converted into the corresponding analog voltage using the reference supply voltage.

![image.png](attachment:7c37c357-9e35-4186-8af7-083c2ea73012:image.png)

This step translates the digital ADC output into a measurable voltage level.

---

## Step 2: Voltage to FSR Resistance Conversion ( R_fsr)

The FSR is connected in a **voltage divider configuration** with a fixed resistor of **10 kΩ**. Using the measured output voltage, the resistance of the FSR is calculated using the voltage divider equation.

This calculation determines the effective resistance of the FSR for each applied load.

![image.png](attachment:553206e8-8b66-4d24-8ce8-7dc00933a45b:image.png)

## Step 3: Resistance to Conductance Conversion (G )

Based on the manufacturer-provided characteristic curves of the FSR, **conductance (G=1/R_fsr) exhibits an approximately linear relationship with applied force**. Since FSR resistance has a nonlinear relationship with force, the resistance value is inverted to obtain conductance, which shows improved linearity with load.

![image.png](attachment:331d49f3-df97-45e0-bd8e-84789fb8108d:image.png)

Conductance values are expressed in **microsiemens (µS)** and are used for deriving calibration models.

## C++ Code for Extracting Electrical Parameters

C++ code is implemented to compute and output the following parameters for a single FSR cell:

- Output voltage  V_out
- FSR resistance  R_fsr
- Conductance G

```cpp
//code to get the values such has V_out ,Conducatance, R_fsr,G. etc 
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
```

## Summary of Calibration Data

The table below represents the averaged calibration results for each reference mass, including:

- Mean ADC value
- Calculated output voltage
- FSR resistance
- Conductance

These parameters serve as inputs for developing calibration equations that map raw ADC readings to physical units such as grams.

This systematic conversion process ensures a **consistent and repeatable relationship** between applied load and sensor output, forming the foundation for the linear calibration model.

### Calibration Data Table

| Reference Mass (g) | Avg ADC | Avg V_out  | Avg R_fsr (Ω) | Avg Conductance G (µS) |
| --- | --- | --- | --- | --- |
| 20 | 50.3 | 0.041 | 827,869 | 1.24 |
| 50 | 154.9 | 0.125 | 254,653 | 3.93 |
| 100 | 308.2 | 0.248 | 125,698 | 8.15 |
| 200 | 639 | 0.515 | 55,048 | 18.54 |
| 500 | 1202.3 | 0.969 | 24,193 | 41.63 |

## Linear Calibration Model Using Conductance

Based on the manufacturer-provided characteristic curves of the FSR, the sensor’s conductance **G=1/R_fsr** shows an approximately linear relationship with applied force. Therefore, raw ADC readings are first converted into conductance before applying calibration models to estimate physical mass.

After converting ADC values into conductance G, mathematical calibration models are used to map these electrical values to physical mass **y (in grams)**. These models account for sensor nonlinearity, scaling, and offset.

## Linear Calibration Equation

The linear calibration approach uses a first-order polynomial model:

                                                      **y=mx+c** 

Where:

- y  = Estimated mass in grams (g)
- x  = Measured conductance in microsiemens (µS)
- m  = Scaling factor (sensor sensitivity)
- c  = Offset or bias term

Conductance is chosen as the input variable because it provides a more linear response to applied force than raw resistance.

## Determination of Calibration Constants

To compute the calibration constants, known reference weights (20 g, 50 g, 100 g, 200 g, and 500 g) were applied to a single sensing cell. For each load, conductance values were calculated and averaged.

The slope m is calculated as:

![image.png](attachment:34451fed-5b37-4813-a2af-efc37e90d52b:image.png)

The intercept c is calculated using:

![image.png](attachment:67c68ec7-3629-4baa-ae50-4828ae672086:image.png)

Where n is the number of calibration points.

For the collected average ADC values, the calculated calibration constants were:

- **m = 11.82**
- **c = 0.171**

## Final Calibration Equation

The final linear calibration equation used in the system is:

                                                             **Mass (g)=11.82×G+0.171**

By converting resistance to conductance, the nonlinear behavior of the FSR is significantly reduced, allowing a simple linear model to achieve reliable accuracy. This approach provides an efficient balance between computational simplicity and measurement accuracy, making it well suited for real-time processing across an FSR matrix.

## C++ Calibration Code

C++ calibration code is implemented to convert the sensor output of a single FSR cell into mass using the linear calibration model **y=mx+c** 

formula 

```cpp
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
```

## Other Calibration Methods

In addition to linear calibration, other commonly used FSR calibration methods include:

### Power Law Calibration

![image.png](attachment:c7311641-8a51-4406-ad95-eebd3c6d78b4:image.png)

### Fourth-Order Polynomial Calibration

![image.png](attachment:9262565c-3f2b-46b6-98e5-011574bafade:image.png)

**A unified C++ calibration code is implemented that integrates linear, power law, and fourth-order polynomial calibration methods for measuring the mass applied to a single FSR cell.**
