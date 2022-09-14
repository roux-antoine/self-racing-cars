#!/usr/bin/python3

import sys
import time

import adafruit_gps
import rospy
import serial

from self_racing_car_msgs.msg import RmcNmea

PI_PORT = "/dev/ttyUSB0"


"""
TODO:
- improve logging
- improve error handling
"""


class GpsPublisher:
    def __init__(self):
        uart = serial.Serial(PI_PORT, baudrate=9600, timeout=10)
        self.gps = adafruit_gps.GPS(uart, debug=False)
        self.pub = rospy.Publisher("gps_info", RmcNmea, queue_size=10)
        rospy.init_node("talker", anonymous=True)
        self.rate = rospy.Rate(1000)  # 1kHz

    def setup_connection(self):

        # Tell to only send RMC (required to get 10 Hz)
        self.gps.send_command(b"PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")

        # Query current NMEA sentence output settings
        self.gps.send_command(b"PMTK414")
        while True:
            print("Waiting for response from GPS")
            if not self.gps.update():
                time.sleep(1)  # sleeping 1s to avoid looping every few microseconds
                continue
            else:
                if "PMTK514" in self.gps._raw_sentence:
                    if (
                        self.gps._raw_sentence
                        == "$PMTK514,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*33"
                    ):
                        print("Command PMTK314 did run successfully")
                        break
                    else:
                        print("Command PMTK314 did not run successfully, exiting")
                        sys.exit()

    def loop(self):
        while not rospy.is_shutdown():

            if not self.gps.update():
                continue
            elif not self.gps.has_fix:
                # TODO maybe we can use this to trigger a fallback behavior later on
                continue
            else:
                msg = RmcNmea()
                msg.timestamp_utc = self.gps.timestamp_utc
                msg.latitude = self.gps.latitude
                msg.longitude = self.gps.longitude
                msg.speed_knots = self.gps.speed_knots
                msg.track_angle = self.gps.track_angle

                print(self.gps._raw_sentence)

                self.pub.publish(msg)
            self.rate.sleep()


if __name__ == "__main__":
    gps_publisher = GpsPublisher()
    gps_publisher.setup_connection()
    gps_publisher.loop()
