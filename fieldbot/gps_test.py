import serial
import pynmea2
import csv
from datetime import datetime

ser = serial.Serial('COM6', 115200, timeout=1)

# Create unique filename with timestamp
filename = datetime.now().strftime("gps_log_%Y%m%d_%H%M%S.csv")

with open(filename, 'w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(['latitude', 'longitude'])

    while True:
        line = ser.readline().decode('utf-8', errors='ignore')

        if line.startswith('$GNGGA') or line.startswith('$GPGGA'):
            try:
                msg = pynmea2.parse(line)

                if msg.latitude and msg.longitude:
                    print(msg.latitude, msg.longitude)
                    writer.writerow([msg.latitude, msg.longitude])

            except:
                pass