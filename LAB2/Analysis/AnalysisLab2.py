#!/usr/bin/env python3
import bagpy
from bagpy import bagreader
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from statistics import stdev

reader1 = bagreader('/home/iso1/Desktop/EECE5554/LAB2/data/open_stat.bag')
reader2 = bagreader('/home/iso1/Desktop/EECE5554/LAB2/data/open_walk.bag')
reader3 = bagreader('/home/iso1/Desktop/EECE5554/LAB2/data/occ_stat.bag')
reader4 = bagreader('/home/iso1/Desktop/EECE5554/LAB2/data/occ_walk.bag')

data1 = reader1.message_by_topic('/gps')
data2 = reader2.message_by_topic('/gps')
data3 = reader3.message_by_topic('/gps')
data4 = reader4.message_by_topic('/gps')

df1 = pd.read_csv(data1)
df2 = pd.read_csv(data2)
df3 = pd.read_csv(data3)
df4 = pd.read_csv(data4)

easting1 = df1["UTM_easting"] - df1["UTM_easting"].min()
northing1 = df1["UTM_northing"] - df1["UTM_northing"].min()
latitude1 = df1["Latitude"]
longitude1 = df1["Longitude"]

easting2 = df2["UTM_easting"] - df2["UTM_easting"].min()
northing2 = df2["UTM_northing"] - df2["UTM_northing"].min()
latitude2 = df2["Latitude"]
longitude2 = df2["Longitude"]

easting3 = df3["UTM_easting"] - df3["UTM_easting"].min()
northing3 = df3["UTM_northing"] - df3["UTM_northing"].min()
latitude3 = df3["Latitude"]
longitude3 = df3["Longitude"]

easting4 = df4["UTM_easting"] - df4["UTM_easting"].min()
northing4 = df4["UTM_northing"] - df4["UTM_northing"].min()
latitude4 = df4["Latitude"]
longitude4 = df4["Longitude"]

#plotting the 2D-Histogram for Open area stationary
plt.hist2d(easting1, northing1, bins=(50, 50), alpha=0.8)
plt.xlabel('Easting')
plt.ylabel('Northing')
plt.title('Open Area Stationary - 2D Histogram')
plt.colorbar()
plt.show()

#plotting the 2D-Histogram for Open area walking
plt.hist2d(easting2, northing2, bins=(50, 50), alpha=0.8)
plt.xlabel('Easting')
plt.ylabel('Northing')
plt.title('Open Area Walking - 2D Histogram')
plt.colorbar()
plt.show()

#plotting the 2D-Histogram for closed area stationary
plt.hist2d(easting3, northing3, bins=(50, 50), alpha=0.8)
plt.xlabel('Easting')
plt.ylabel('Northing')
plt.title('Occluded Area Stationary - 2D Histogram')
plt.colorbar()
plt.show()

#plotting the 2D-Histogram for closed area walking
plt.hist2d(easting4, northing4, bins=(50, 50), alpha=0.8)
plt.xlabel('Easting')
plt.ylabel('Northing')
plt.title('Occluded Area Walking - 2D Histogram')
plt.colorbar()
plt.show()
