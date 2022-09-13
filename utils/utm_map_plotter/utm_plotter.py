import glob
import os

import matplotlib.pyplot as plt
import numpy as np
import utm


class NMEA:
    def __init__(self, sentence: str):
        self.sentence = sentence
        self.latitude = self.compute_latitude()
        self.longitude = self.compute_longitude()

    def compute_latitude(self):
        latitude_raw = self.sentence.split(",")[3]
        latitude_degree = latitude_raw[0:2]
        latitude_minutes = latitude_raw[2:]
        latitude_sign = "+" if self.sentence.split(",")[4] == "N" else "-"
        latitude_decimal_unsigned = (
            float(latitude_degree) + float(latitude_minutes) / 60
        )
        latitude_decimal = float(f"{latitude_sign}{latitude_decimal_unsigned}")

        return latitude_decimal

    def compute_longitude(self):
        longitude_raw = self.sentence.split(",")[5]
        longitude_degree = longitude_raw[0:3]
        longitude_minutes = longitude_raw[3:]
        longitude_sign = "+" if self.sentence.split(",")[6] == "E" else "-"
        longitude_decimal_unsigned = (
            float(longitude_degree) + float(longitude_minutes) / 60
        )
        longitude_decimal = float(f"{longitude_sign}{longitude_decimal_unsigned}")

        return longitude_decimal


flist = glob.glob(os.path.join("lat_lon_files", "*.txt"))
print("Available txt: ")
for path in flist:
    print(path)
print()


input_name_map = input("Enter name for input_name_map: ")

if input_name_map:

    map_latitudes = []
    map_longitudes = []

    with open(input_name_map) as input_file:
        for line in input_file:
            map_latitudes.append(float(line.split()[1]))
            map_longitudes.append(float(line.split()[0]))

    map_utms = utm.from_latlon(np.array(map_latitudes), np.array(map_longitudes))

    plt.plot(map_utms[:][0], map_utms[:][1])

###

flist = glob.glob(
    os.path.join("gps_recordings/recordings_biking_and_garden", "*.nmea"),
    recursive=True,
)
print("Available nmea: ")
for path in flist:
    print(path)
print()

input_name_nmea = input("Enter name for nmea: ")

if input_name_nmea:
    nmeas = []
    with open(input_name_nmea) as logfile:
        for line in logfile.readlines():
            if "GNRMC" in line and line.split(",")[2] == "A":
                nmeas.append(NMEA(line))

    gps_latitudes = [x.latitude for x in nmeas]
    gps_longitudes = [x.longitude for x in nmeas]

    gps_utms = utm.from_latlon(np.array(gps_latitudes), np.array(gps_longitudes))

    plt.plot(gps_utms[:][0], gps_utms[:][1])

plt.axis("equal")
plt.show()
