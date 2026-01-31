import serial
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# --- Settings ---
COM_PORT = 'COM6'
BAUD_RATE = 921600
ROWS, COLS = 32, 96  # 3 sensors of 32 columns each
EXPECTED_SAMPLES = 3072 # 32 * 32 * 3

try:
    ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=0.1)
    ser.flushInput()
    print(f"Connected to {COM_PORT}")
except Exception as e:
    print(f"Error: {e}")
    exit()

fig, ax = plt.subplots(figsize=(15, 5))
data_matrix = np.zeros((ROWS, COLS))
im = ax.imshow(data_matrix, cmap='jet', interpolation='gaussian', vmin=0, vmax=1023)
plt.colorbar(im)

def update(frame):
    if ser.in_waiting > 0:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not line: return [im]
            
            raw_data = [int(x) for x in line.split(',') if x.strip()]
            
            if len(raw_data) == EXPECTED_SAMPLES:
                # Reshape data into three 32x32 tiles
                # Note: Arduino sends RowA,B then C,D then E,F per Mux step
                all_data = np.array(raw_data).reshape(32, 16, 6)
                
                tile1 = all_data[:, :, 0:2].reshape(32, 32).T
                tile2 = all_data[:, :, 2:4].reshape(32, 32).T
                tile3 = all_data[:, :, 4:6].reshape(32, 32).T

                # Stitch them together
                full_canvas = np.hstack([tile1, tile2, tile3])
                
                # Apply simple noise threshold
                full_canvas[full_canvas < 40] = 0
                im.set_data(full_canvas)
            else:
                print(f"Buffer Error: Received {len(raw_data)} / {EXPECTED_SAMPLES}")
                
        except Exception as e:
            print(f"Stream error: {e}")
    return [im]

ani = FuncAnimation(fig, update, interval=1, blit=True, cache_frame_data=False)
plt.show()
ser.close()
