# import serial

# port = '/dev/ttyUSB0'

# ser = serial.Serial(port, baudrate=115200, timeout=1)

# def parse_gps_data(data):
#     if data.startswith(''):
#         parts = data.split('$VNYMR')
#         return parts
        
# while True:
#     line = ser.readline().decode('ascii', errors='replace').strip()
#     print(line)
#     #gps_coords = parse_gps_data(line)
#     #if gps_coords:
#         #print(gps_coords)


# # VNYMR data
# #  0         1          2          3           4          5          6         7          8         9         10           11            12
# #$VNYMR,  -131.192,  +031.896,  +008.488,  -00.2816,  +00.2568,  -00.1372,  +05.354,  -01.221,  -08.319,  -00.013897,  +00.053735,  +00.072580*62

# #Accelerometer (acc) values (X, Y, Z)
# #Gyroscope (gyro) values (X, Y, Z)
# #Orientation data (roll, pitch, yaw)

# # [0]
# # [1]  yaw
# # [2]  pitch
# # [3]  roll
# # [4]  magnetic field x
# # [5]  magnetic field y
# # [6]  magnetic field z
# # [7]  acceleration x
# # [8]  acceleration y
# # [9] acceleration z
# # [10] angular vel x
# # [11] angular vel y
# # [12] angular vel z

import matplotlib.pyplot as plt
import numpy as np

# Create some sample data
data = np.random.normal(0, 1, 1000)

# Plot the histogram with edgecolor
plt.hist(data, bins=30, edgecolor='black')

# Show the plot
plt.show()