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
