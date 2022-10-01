#!/usr/bin/python3


import math

import rospy
import utm

from self_racing_car_msgs.msg import RmcNmea, VehicleState


class VehicleStatePublisher:
    def __init__(self):
        rospy.init_node("vehicle_state_publisher", anonymous=True)

        self.sub = rospy.Subscriber("gps_info", RmcNmea, self.callback, queue_size=10)
        self.pub = rospy.Publisher("vehicle_state", VehicleState, queue_size=10)
        self.rate = rospy.Rate(1000)  # 1kHz

        self.last_state = None
        self.last_msg_seq = None
        self.JUMPING_MESSAGE_FACTOR = 3

    def callback(self, rmc_msg):

        # small logic to skip messages if we want
        if self.last_msg_seq:
            if rmc_msg.header.seq - self.last_msg_seq < self.JUMPING_MESSAGE_FACTOR:
                return
            else:
                self.last_msg_seq = rmc_msg.header.seq
        else:
            self.last_msg_seq = rmc_msg.header.seq

        # TODO handle the situation where the location is exactly the same as in the previous message
        # in this case, the best is probably to not publish

        # read RMC message
        latitude = rmc_msg.latitude
        longitude = rmc_msg.longitude

        # convert to utm
        utm_values = utm.from_latlon(latitude, longitude)

        # build the vehicle state message
        vehicle_state_msg = VehicleState()
        vehicle_state_msg.x = utm_values[0]
        vehicle_state_msg.y = utm_values[1]
        vehicle_state_msg.z = 0
        vehicle_state_msg.vx = -1  # TODO project the speed
        vehicle_state_msg.vy = -1  # TODO project the speed
        vehicle_state_msg.vz = -1  # TODO project the speed

        # computing the angle with a difference
        # if self.last_state:
        #     diff_x = vehicle_state_msg.x - self.last_state.x
        #     diff_y = vehicle_state_msg.y - self.last_state.y
        #     angle = math.pi / 2 - math.atan2(
        #         diff_y, -diff_x
        #     )  # minus sign is a bit of a hack, probably because of the utm coordinates
        #     # TODO add some filtering here, but make sure we account for the fact that it loops at 2 pi!
        #     vehicle_state_msg.angle = angle
        # else:
        #     vehicle_state_msg.angle = 0

        # using the track angle degree for the angle
        angle = -rmc_msg.track_angle_deg * math.pi / 180
        vehicle_state_msg.angle = angle
        self.pub.publish(vehicle_state_msg)

        self.last_state = vehicle_state_msg


if __name__ == "__main__":
    vehicle_state_publisher = VehicleStatePublisher()
    rospy.spin()
