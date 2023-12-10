import numpy as np
import math
import pandas as pd
from bagpy import bagreader
import matplotlib.pyplot as plt

b = bagreader('/home/iso1/Desktop/EECE5554/LAB1/src/Data/walking.bag')
GPS_MSG = b.message_by_topic('/gps')
Reader = pd.read_csv("/home/iso1/Desktop/EECE5554/LAB1/src/Data/walking/gps.csv")

def distance(lat1, lon1, lat2, lon2):
    # convert decimal degrees to radians 
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])

    # haversine formula 
    dlat = lat2 - lat1 
    dlon = lon2 - lon1 
    a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2

    # Ensure that 'a' is within the valid range
    a = np.clip(a, -1, 1)

    # Handle the case where 'a' is slightly outside the valid domain
    try:
        c = 2 * math.asin(math.sqrt(a)) 
    except ValueError:
        # Set c to a default value or handle the error as needed
        c = 0

    r = 6371 # Radius of earth in kilometers.
    return c * r

# Error calculations
k_lat_start = 42.205000
k_lat_end = 42.2045446

k_lon_start = -71.058514
k_lon_end = -71.0592692

k_lats = []
k_lons = []

lat_interval = (k_lat_end - k_lat_start) / 67
lon_interval = (k_lon_end - k_lon_start) / 67

for i in range(0, 67):
    k_lats.append(k_lat_start + (i * lat_interval))
    k_lons.append(k_lon_start + (i * lon_interval))

error = []
for i in range(0, 67):
    error.append(distance(Reader["Latitude"][i], Reader["Longitude"][i], k_lats[i], k_lons[i]))

plt.hist(error, bins=30)    
plt.show()
