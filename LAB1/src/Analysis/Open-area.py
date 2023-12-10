import numpy as np

import matplotlib.pyplot as plt
import pandas as pd
import math

def distance(lat1, lon1, lat2, lon2):
    # convert decimal degrees to radians 
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])

    # haversine formula 
    dlat = lat2 - lat1 
    dlon = lon2 - lon1 
    a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
    c = 2 * math.asin(math.sqrt(a)) 
    r = 6371 # Radius of earth in kilometers.
    return c * r


# Load the data from a LibreOffice spreadsheet into a pandas DataFrame
df = pd.read_csv('/home/iso1/Desktop/EECE5554/LAB1/src/Analysis/Open.csv')



# Subtract the first value from each column to scale the data with your stationary data sets
easting = df['UTM_easting'] = df['UTM_easting'] - df['UTM_easting'][0]
northing = df['UTM_northing'] = df['UTM_northing'] - df['UTM_northing'][0]
altitude = df['Altitude']
time = df['Time']
latitude = df['Latitude']
longitude = df['Longitude']

# Plotting the scatterplot of northing vs easting data
plt.scatter(easting, northing)
plt.xlabel('Easting')
plt.ylabel('Northing')
plt.title('Scatterplot of open-area Northing vs Easting Data')
plt.show()

# Plotting the scatterplot of Altitude vs Time data
plt.scatter(time, altitude)
plt.xlabel('Time')
plt.ylabel('Altitude')
plt.title('Scatterplot of open-area  Altitude vs Time Data')
plt.show()

# Error calculations
known_lat_start = 42.205000
known_lat_end = 42.2045446

known_lon_start = -71.058514
known_lon_end = -71.0592692


known_lats = []
known_lons = []

lat_interval = (known_lat_end - known_lat_start) / 67
lon_interval = (known_lon_end - known_lon_start) / 67

for i in range(0,67):
    known_lats.append(known_lat_start + (i * lat_interval))
    known_lons.append(known_lon_start + (i * lon_interval))

error = []
for i in range(0,67):
    error.append(distance(latitude[i], longitude[i], known_lats[i], known_lons[i]))

# Error histogram plot
plt.hist(error, bins=20)
plt.xlabel('Error(Kms)')
plt.ylabel('Frequency')
plt.title('Error of open-area data. Frequency vs Error data')
plt.show()

# Error vs Time plot
plt.scatter(time,error)
plt.xlabel('Time')
plt.ylabel('Error (kms)')
plt.title('Error vs Time plot for Open-area')
plt.show()

me = np.mean(error)
print(me) 
