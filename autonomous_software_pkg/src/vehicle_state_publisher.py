#!/usr/bin/python3


import rospy
import utm

from self_racing_car_msgs.msg import RmcNmea, VehicleState


class VehicleStatePublisher:
    def __init__(self):
        rospy.init_node("vehicle_state_publisher", anonymous=True)

        self.sub = rospy.Subscriber("gps_info", RmcNmea, self.callback, queue_size=10)
        self.pub = rospy.Publisher("vehicle_state", VehicleState, queue_size=10)
        self.rate = rospy.Rate(1000)  # 1kHz

    def callback(self, rmc_msg):
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
        vehicle_state_msg.track_angle_deg = rmc_msg.track_angle_deg

        self.pub.publish(vehicle_state_msg)

    def loop(self):
        while not rospy.is_shutdown():
            # print("hello")
            self.rate.sleep()


if __name__ == "__main__":
    vehicle_state_publisher = VehicleStatePublisher()
    # vehicle_state_publisher.loop()
    rospy.spin()
