#!/usr/bin/python3

import numpy as np
import rospy
import utm
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation

from self_racing_car_msgs.msg import RmcNmea

MAP_PATH = "/home/antoine/workspace/catkin_ws/src/utils/utm_map_plotter/lat_lon_files/rex_manor_full.txt"


class RealTimePlotter:
    def __init__(self):
        self.last_lat = None
        self.last_lon = None
        self.last_point = [None, None]
        self.previous_points = []

    def prepare_map(self):
        tmp_latitudes = []
        tmp_longitudes = []

        list_latitudes = []
        list_longitudes = []
        list_colors = []
        list_styles = []

        with open(MAP_PATH) as input_file:
            for line in input_file:
                if line == "\n":
                    list_latitudes.append(tmp_latitudes)
                    list_longitudes.append(tmp_longitudes)
                    tmp_latitudes = []
                    tmp_longitudes = []
                elif line == "edge\n":
                    list_colors.append("black")
                    list_styles.append("plot")
                elif line == "refline\n":
                    list_colors.append("blue")
                    list_styles.append("scatter")
                else:
                    tmp_latitudes.append(float(line.split()[1]))
                    tmp_longitudes.append(float(line.split()[0]))
            list_latitudes.append(tmp_latitudes)
            list_longitudes.append(tmp_longitudes)

        for lats, lons, color, style in zip(
            list_latitudes, list_longitudes, list_colors, list_styles
        ):
            map_utms = utm.from_latlon(np.array(lats), np.array(lons))
            if style == "plot":
                plt.plot(map_utms[:][0], map_utms[:][1], color=color)
            elif style == "scatter":
                plt.scatter(map_utms[:][0], map_utms[:][1], color=color)
            else:
                print("Error...")

    def plotting_callback(self, msg):
        gps_utms = utm.from_latlon(msg.latitude, np.array(msg.longitude))
        self.last_lat = gps_utms[0]
        self.last_lon = gps_utms[1]
        self.last_point = [gps_utms[0], gps_utms[1]]

    def animate(self, i):
        if self.last_point not in self.previous_points:
            self.previous_points.append(self.last_point)
            plt.scatter(self.last_point[0], self.last_point[1])


if __name__ == "__main__":
    real_time_plotter = RealTimePlotter()
    real_time_plotter.prepare_map()
    rospy.init_node("real_time_plotter")
    rospy.Subscriber("gps_info", RmcNmea, real_time_plotter.plotting_callback)
    ani = FuncAnimation(plt.gcf(), real_time_plotter.animate, interval=50)
    plt.show()
