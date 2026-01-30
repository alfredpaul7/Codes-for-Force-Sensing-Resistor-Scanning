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
