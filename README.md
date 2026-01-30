# Documentation for Interfacing and data Acquisation of  Force sensing resistor.

***You can also view this documentation on the web or on Notion. The links are provided below. The project report can be accessed using the link given.***

**WEB Link :** [https://laced-cap-19e.notion.site/Documentation-for-Interfacing-and-data-Acquisation-of-Force-sensing-resistor-2f835f708a9380aaaa4dd4aeb9c6d309?source=copy_link](https://www.notion.so/Documentation-for-Interfacing-and-data-Acquisation-of-Force-sensing-resistor-2f835f708a9380aaaa4dd4aeb9c6d309?pvs=21)

**Notion link:** [https://www.notion.so/Documentation-for-Interfacing-and-data-Acquisation-of-Force-sensing-resistor-2f835f708a9380aaaa4dd4aeb9c6d309?source=copy_link](https://www.notion.so/Documentation-for-Interfacing-and-data-Acquisation-of-Force-sensing-resistor-2f835f708a9380aaaa4dd4aeb9c6d309?pvs=21)

**Report link :** https://drive.google.com/drive/folders/1ozw_Ty0pGpx6KO5M5wNeE8qQ3ilpqTL_?usp=sharing

**Github repository:**

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
