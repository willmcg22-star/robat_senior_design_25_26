import pandas as pd
import matplotlib.pyplot as plt

data = pd.read_csv('gps_log_20260413_172858.csv')

lat = data['latitude']
lon = data['longitude']

plt.scatter(lon, lat)
plt.xlabel("Longitude")
plt.ylabel("Latitude")
plt.title("GPS Path Tracking Demo")
plt.axis('equal')
plt.show()