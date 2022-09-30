#!/usr/bin/python3

import math

import matplotlib.pyplot as plt
import numpy as np
import rospy
from matplotlib.animation import FuncAnimation

from self_racing_car_msgs.msg import VehicleCommand, VehicleState


class Waypoint:
    def __init__(self, id, x, y, z):
        self.id = id
        self.x = x
        self.y = y
        self.z = z


class Controller:
    def __init__(self):

        # Parameters
        topic_current_state = rospy.get_param("~topic_current_state", "vehicle_state")
        self.lookahead_distance = rospy.get_param("~lookahead_distance", 2.5)
        self.wheel_base = rospy.get_param("~wheel_base", 1)
        self.max_curvature = rospy.get_param("~max_curvature", 100000)
        self.min_curvature = rospy.get_param("~min_curvature", 0.3)
        # self.frequency          = rospy.get_param('~frequency', 2.0)
        # self.rate               = rospy.Rate(self.frequency)
        # self.rate_init          = rospy.Rate(1.0)   # Rate while we wait for topic
        wp_file = "/home/antoine/workspace/catkin_ws/src/utils/utm_map_generation/x_y_files/grattan_street_waypoints.txt"
        # edges_file = "/home/mattia/catkin_ws/src/controller/src/grattan_edges.txt"

        self.PAST_STATES_WINDOW_SIZE = 5
        self.X_AXIS_LIM = 10
        self.Y_AXIS_LIM = 10
        self.WAYPOINTS_BEHIND_NBR = 3
        self.WAYPOINTS_AFTER_NBR = 5
        self.COLOR_CYCLE = plt.rcParams["axes.prop_cycle"].by_key()["color"]

        # Subscribers
        rospy.Subscriber(topic_current_state, VehicleState, self.callback_current_state)

        # Publishers
        self.pub_vehicle_cmd = rospy.Publisher(
            "vehicle_cmd", VehicleCommand, queue_size=10
        )

        # Initial values
        self.current_state = VehicleState(0, 0, 0, 0, 0, 0, 0)
        self.past_n_states = []

        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111)

        # Test
        TEST_FAKE_WAYPOINTS = False

        if TEST_FAKE_WAYPOINTS:
            self.current_waypoints = [
                Waypoint(0, 0, 0, 0),
                Waypoint(1, 0, 1, 0),
                Waypoint(2, 0, 2, 0),
                Waypoint(3, 0, 3, 0),
                Waypoint(4, 0, 4, 0),
                Waypoint(5, 0, 5, 0),
                Waypoint(6, 0, 6, 0),
            ]
            self.current_state = VehicleState()
            self.current_state.x = 1
            self.current_state.y = 0
            self.current_state.z = 0.0
            self.current_state.track_angle_deg = 0

        else:
            self.waypoints_xs, self.waypoints_ys = self.load_waypoints(wp_file)

            self.current_waypoints = []
            for i in range(len(self.waypoints_xs)):
                self.current_waypoints.append(
                    Waypoint(i, self.waypoints_xs[i], self.waypoints_ys[i], 0)
                )

            # plt.scatter(waypoints_xs, waypoints_ys)

            # for id, x, y in zip(range(len(waypoints_xs)), waypoints_xs, waypoints_ys):
            #     plt.annotate(id, (x, y))

            # self.edges_xs_list, self.edges_ys_list = self.load_edges(edges_file)
            self.edges_xs_list, self.edges_ys_list = [], []

    # ##################### PURE PURSUIT FUNCTIONS #####################

    def callback_current_state(self, msg):

        self.current_state = msg
        self.past_n_states.append(self.current_state)
        if len(self.past_n_states) > self.PAST_STATES_WINDOW_SIZE:
            self.state_to_unscatter = self.past_n_states.pop(0)

        # while (self.current_waypoints == None or self.current_state == None) and not rospy.is_shutdown():
        #     rospy.logwarn('Waiting for topics ...')
        #     self.rate_init.sleep()
        #
        # rospy.logwarn('subscribed to topics - starting')
        #
        # while not rospy.is_shutdown():
        if True:

            # Add something to only run if we received input topics recently

            # Find lookahead waypoint = First waypoint further than lookahead distance and in front of vehicle
            self.lookahead_wp, self.id_lookahead_wp = self.getNextWaypoint()

            # print("Lookahead wp: ")
            # print("(x, y): ", str(lookahead_wp.x) + ", " + str(lookahead_wp.y))

            # print("Current pose: ")
            # print(
            #     "(x, y, yaw): ",
            #     str(self.current_state.x)
            #     + ", "
            #     + str(self.current_state.y)
            #     + ", "
            #     + str(self.current_state.track_angle_deg),
            # )

            # Get target waypoint - Linear interpolation between lookahead waypoint and waypoint before it
            # Equation linear equation: "ax + by + c = 0"
            # if there are two points (x1,y1) , (x2,y2), a = "y2-y1, b = "(-1) * x2 - x1" ,c = "(-1) * (y2-y1)x1 + (x2-x1)y1"
            a, b, c = self.getLinearEquation(
                self.current_waypoints[self.id_lookahead_wp],
                self.current_waypoints[self.id_lookahead_wp - 1],
            )
            # print("Line equation: ", a, b, c)

            # Compute distance to line
            # d = self.getDistanceBetweenLineAndPoint(a, b, c)
            # print 'Distance to line: ', d

            # Find intersection between line and circle around vehicle
            intersections = self.circle_line_segment_intersection(
                (self.current_state.x, self.current_state.y),
                self.lookahead_distance,
                (
                    self.current_waypoints[self.id_lookahead_wp].x,
                    self.current_waypoints[self.id_lookahead_wp].y,
                ),
                (
                    self.current_waypoints[self.id_lookahead_wp - 1].x,
                    self.current_waypoints[self.id_lookahead_wp - 1].y,
                ),
            )
            # print("Intersections: ", intersections)

            if len(intersections) == 2:
                # print("2 intersections")
                # Choose the waypoint that is closer to the lookahead wp
                d1 = np.sqrt(
                    (
                        (intersections[0][0] - self.lookahead_wp.x) ** 2
                        + (intersections[0][1] - self.lookahead_wp.y) ** 2
                    )
                )
                d2 = np.sqrt(
                    (
                        (intersections[1][0] - self.lookahead_wp.x) ** 2
                        + (intersections[1][1] - self.lookahead_wp.y) ** 2
                    )
                )

                if d1 < d2:
                    # Set the target waypoint id as -1
                    self.target_wp = Waypoint(
                        -1, intersections[0][0], intersections[0][1], 0
                    )
                else:
                    self.target_wp = Waypoint(
                        -1, intersections[1][0], intersections[1][1], 0
                    )

            elif len(intersections) == 1:
                # If only one intersection, maybe the target waypoint should be the lookahead waypoint
                # print("1 intersection")
                self.target_wp = self.lookahead_wp

            else:
                # print("0 intersection")
                self.target_wp = self.lookahead_wp

            # print("target_wp: ", target_wp)

            # Compute curvature between vehicle and target waypoint
            self.curvature = self.ComputeCurvature(self.target_wp)

            # print("radius: ", 1 / curvature)
            # print("curvature: ", curvature)
            #
            # Convert curvature into steering angle
            self.steering_angle = self.ConvertCurvatureToSteeringAngle(self.curvature)

            # Publish cmd
            # self.publish_vehicle_cmd(steering_angle)

            # print("Steering: ", steering_angle)

            show_plot = False

            if show_plot:
                pass
                # car_vector = np.array(
                #     [
                #         self.wheel_base
                #         * np.cos(self.current_state.track_angle_deg + np.pi / 2),
                #         self.wheel_base
                #         * np.sin(self.current_state.track_angle_deg + np.pi / 2),
                #     ]
                # )

                # car2target_vector = np.array(
                #     [
                #         (self.target_wp.x - self.current_state.x),
                #         self.target_wp.y - self.current_state.y,
                #     ]
                # )

                # # Car's direction
                # plt.arrow(
                #     self.current_state.x,
                #     self.current_state.y,
                #     car_vector[0],
                #     car_vector[1],
                #     head_width=0.03,
                #     head_length=0.1,
                #     length_includes_head=True,
                #     width=0.01,
                #     color="blue",
                # )

                # # Car 2 target direction
                # plt.arrow(
                #     self.current_state.x,
                #     self.current_state.y,
                #     car2target_vector[0],
                #     car2target_vector[1],
                #     head_width=0.03,
                #     head_length=0.1,
                #     length_includes_head=True,
                #     width=0.01,
                #     color="green",
                # )

                # plt.legend()
                # plt.axis("equal")
                # plt.grid()
                # plt.show()

        # self.rate.sleep()

    def getNextWaypoint(self):
        # Returns the first waypoint that is further than lookahead distance

        # Find the closest waypoint
        d_min = 10000
        wp_closest_id = 0

        for wp in self.current_waypoints:
            d = self.getPlaneDistance(wp, self.current_state)
            if d < d_min:
                d_min = d
                wp_closest_id = wp.id

        # Iterate through all the waypoints starting from the closest + 1
        for wp in self.current_waypoints[wp_closest_id + 1 :]:
            if self.getPlaneDistance(wp, self.current_state) > self.lookahead_distance:
                return wp, wp.id

    def getPlaneDistance(self, current_wp, current_state):
        # Inputs:
        #   - current_wp (type Waypoint)
        #   - current_state (type VehicleState)
        # Output:
        #   - Plane distance between the input waypoint and input state
        return math.sqrt(
            (current_wp.x - current_state.x) ** 2
            + (current_wp.y - current_state.y) ** 2
        )

    def compute_relative_xy_offset(self, target, current_pose):
        # Inputs:
        #   - target (type Waypoint)
        #   - current_pose (type VehicleState)
        # Ouput:# Find intersection between the line and the circle

        # Case 1: 2 intersections

        # Case 2: 0 intersections
        # print 'interpolate'
        #   - Relative offset x

        diff_x = target.x - current_pose.x
        diff_y = target.y - current_pose.y
        yaw = current_pose.track_angle_deg

        cos_pose = math.cos(yaw)
        sin_pose = math.sin(yaw)

        relative_x = cos_pose * diff_x + sin_pose * diff_y

        return relative_x

    def ComputeCurvature(self, target):
        # Inputs:
        #   - current_state (type VehicleState)
        #   - target_wp (type Waypoint)
        # Ouput:
        #   - Curvature of the circle arc between the two points

        car_vector = np.array(
            [
                self.wheel_base
                * np.cos(self.current_state.track_angle_deg + np.pi / 2),
                self.wheel_base
                * np.sin(self.current_state.track_angle_deg + np.pi / 2),
            ]
        )

        car2target_vector = np.array(
            [(target.x - self.current_state.x), target.y - self.current_state.y]
        )

        sign = np.sign(np.cross(car_vector, car2target_vector))

        numerator = (
            sign * 2 * abs(self.compute_relative_xy_offset(target, self.current_state))
        )

        denominator = (self.getPlaneDistance(target, self.current_state)) ** 2

        # print("num: ", numerator)
        # print("deno: ", denominator)
        # print("sign, ", sign)

        if denominator != 0:
            return numerator / denominator

        else:
            if numerator > 0:
                return self.min_curvature
            else:
                return self.max_curvature

    def ConvertCurvatureToSteeringAngle(self, curvature):
        # Input:
        #   - curvature
        # Output:
        #   - steering wheel angle (radians) using a simple bicycle model

        return np.arctan(curvature * self.wheel_base)

    def getLinearEquation(self, pt_a, pt_b):

        a = abs(pt_b.y - pt_a.y)
        b = pt_a.x - pt_b.x
        c = -1 * (pt_b.y - pt_a.y) * pt_a.x + (pt_b.x - pt_a.x) * pt_a.y

        return a, b, c

    def getDistanceBetweenLineAndPoint(self, a, b, c):
        x, y = self.current_state.x, self.current_state.y

        num = abs(a * x + b * y + c)
        den = np.sqrt(a**2 + b**2)

        return num / den

    def circle_line_segment_intersection(
        self, circle_center, circle_radius, pt1, pt2, full_line=True, tangent_tol=1e-9
    ):
        """Find the points at which a circle intersects a line-segment.  This can happen at 0, 1, or 2 points.

        :param circle_center: The (x, y) location of the circle center
        :param circle_radius: The radius of the circle
        :param pt1: The (x, y) location of the first point of the segment
        :param pt2: The (x, y) location of the second point of the segment
        :param full_line: True to find intersections along full line - not just in the segment.  False will just return intersections within the segment.
        :param tangent_tol: Numerical tolerance at which we decide the intersections are close enough to consider it a tangent
        :return Sequence[Tuple[float, float]]: A list of length 0, 1, or 2, where each element is a point at which the circle intercepts a line segment.

        Note: We follow: http://mathworld.wolfram.com/Circle-LineIntersection.html
        """

        (p1x, p1y), (p2x, p2y), (cx, cy) = pt1, pt2, circle_center
        (x1, y1), (x2, y2) = (p1x - cx, p1y - cy), (p2x - cx, p2y - cy)
        dx, dy = (x2 - x1), (y2 - y1)
        dr = (dx**2 + dy**2) ** 0.5
        big_d = x1 * y2 - x2 * y1
        discriminant = circle_radius**2 * dr**2 - big_d**2

        if discriminant < 0:  # No intersection between circle and line
            return []
        else:  # There may be 0, 1, or 2 intersections with the segment
            intersections = [
                (
                    cx
                    + (
                        big_d * dy
                        + sign * (-1 if dy < 0 else 1) * dx * discriminant**0.5
                    )
                    / dr**2,
                    cy + (-big_d * dx + sign * abs(dy) * discriminant**0.5) / dr**2,
                )
                for sign in ((1, -1) if dy < 0 else (-1, 1))
            ]  # This makes sure the order along the segment is correct
            if (
                not full_line
            ):  # If only considering the segment, filter out intersections that do not fall within the segment
                fraction_along_segment = [
                    (xi - p1x) / dx if abs(dx) > abs(dy) else (yi - p1y) / dy
                    for xi, yi in intersections
                ]
                intersections = [
                    pt
                    for pt, frac in zip(intersections, fraction_along_segment)
                    if 0 <= frac <= 1
                ]
            if (
                len(intersections) == 2 and abs(discriminant) <= tangent_tol
            ):  # If line is tangent to circle, return just one point (as both intersections have same location)
                return [intersections[0]]
            else:
                return intersections

    # ##################### MISC #####################

    def publish_vehicle_cmd(self, steering_angle):
        cmd_msg = VehicleCommand()
        cmd_msg.steering_angle = steering_angle
        cmd_msg.throttle_angle = 0

        self.pub_vehicle_cmd.publish(cmd_msg)

    def load_waypoints(self, file):

        with open(file) as waypoints_file:
            waypoints_xs_ys = [
                [float(line.split()[0]), float(line.split()[1])]
                for line in waypoints_file.readlines()
            ]

        waypoints_xs = np.array(np.array(waypoints_xs_ys)[:, 0])
        waypoints_ys = np.array(np.array(waypoints_xs_ys)[:, 1])

        return waypoints_xs, waypoints_ys

    def load_edges(self, file):
        edges_xs_list = []
        edges_ys_list = []
        tmp_xs = []
        tmp_ys = []

        file = open(file, "r")

        for line in file:
            if line != "\n":
                tmp_xs.append(float(line.split()[0]))
                tmp_ys.append(float(line.split()[1]))
            else:
                edges_xs_list.append(tmp_xs)
                edges_ys_list.append(tmp_ys)
                tmp_xs = []
                tmp_ys = []
            edges_xs_list.append(tmp_xs)
            edges_ys_list.append(tmp_ys)

        return edges_xs_list, edges_ys_list

    # ##################### PLOTTING UTILITIES #####################

    def plot_car_frame(self):
        self.ax.arrow(
            self.current_state.x,
            self.current_state.y,
            np.cos(self.current_state.track_angle_deg),
            np.sin(self.current_state.track_angle_deg),
            head_width=0.03,
            head_length=0.1,
            length_includes_head=True,
            width=0.01,
            color="black",
        )
        self.ax.arrow(
            self.current_state.x,
            self.current_state.y,
            np.cos(self.current_state.track_angle_deg + math.pi / 2),
            np.sin(self.current_state.track_angle_deg + math.pi / 2),
            head_width=0.03,
            head_length=0.1,
            length_includes_head=True,
            width=0.01,
            color="black",
        )

    def plot_trajectory_cicle(self):
        x_circle = self.current_state.x - (1 / self.curvature) * np.cos(
            self.current_state.track_angle_deg
        )
        y_circle = self.current_state.y - (1 / self.curvature) * np.sin(
            self.current_state.track_angle_deg
        )

        trajectory_circle = plt.Circle(
            (x_circle, y_circle),
            (1 / self.curvature),
            color="b",
            fill=False,
            label="Arc to follow",
        )
        self.ax.add_patch(trajectory_circle)

    def plot_steering_angle(self):
        self.ax.arrow(
            self.current_state.x,
            self.current_state.y,
            np.cos(self.steering_angle + np.pi / 2 + self.current_state.track_angle_deg)
            / 5,
            np.sin(self.steering_angle + np.pi / 2 + self.current_state.track_angle_deg)
            / 5,
            head_width=0.03,
            head_length=0.1,
            length_includes_head=True,
            width=0.01,
            color="red",
        )

    def prepare_map(self):

        self.ax.scatter(self.waypoints_xs, self.waypoints_ys)

        for id, x, y in zip(
            list(range(len(self.waypoints_xs))), self.waypoints_xs, self.waypoints_ys
        ):
            self.ax.annotate(id, (x, y))

        for edges_xs, edges_ys in zip(self.edges_xs_list, self.edges_ys_list):
            self.ax.plot(edges_xs, edges_ys)

    def animate(self, i):
        if self.current_state.x != 0 and self.current_state.y != 0:
            self.ax.scatter(self.current_state.x, self.current_state.y)

            # Car frame direction
            self.plot_car_frame()

        self.last_x = self.current_state.x
        self.last_y = self.current_state.y
        self.last_track_angle_deg = self.current_state.track_angle_deg

    def plot_states_etc(self, _):

        if self.current_state.x != 0 and self.current_state.y != 0:
            plt.cla()

            # Previous car locations
            for idx, past_state in reversed(list(enumerate(self.past_n_states[:-1]))):
                self.ax.scatter(
                    past_state.x, past_state.y, color=self.COLOR_CYCLE[idx + 1]
                )

            # Car location, car frame and steering angle
            self.ax.scatter(
                self.current_state.x, self.current_state.y, color=self.COLOR_CYCLE[0]
            )
            self.plot_car_frame()
            self.plot_steering_angle()

            # Lookahead waypoint
            self.ax.scatter(
                self.lookahead_wp.x,
                self.lookahead_wp.y,
                color="k",
                label="lookahead waypoint",
            )
            # Target waypoint
            self.ax.scatter(
                self.target_wp.x, self.target_wp.y, color="k", label="target waypoint"
            )

            # Lookahead circle
            lookahead_circle = plt.Circle(
                (self.current_state.x, self.current_state.y),
                self.lookahead_distance,
                color="k",
                linestyle="--",
                fill=False,
                label="Lookahead radius around car",
            )
            self.ax.add_patch(lookahead_circle)

            # Trajectory circle
            self.plot_trajectory_cicle()

            # Waypoints
            for idx in range(
                self.id_lookahead_wp - self.WAYPOINTS_BEHIND_NBR,
                self.id_lookahead_wp + self.WAYPOINTS_AFTER_NBR,
            ):
                self.ax.scatter(
                    self.current_waypoints[idx].x,
                    self.current_waypoints[idx].y,
                    color="k",
                )

            self.ax.legend()
            self.ax.axis("equal")
            self.ax.set_xlim(
                self.current_state.x - self.X_AXIS_LIM,
                self.current_state.x + self.X_AXIS_LIM,
            )
            self.ax.set_ylim(
                self.current_state.y - self.Y_AXIS_LIM,
                self.current_state.y + self.Y_AXIS_LIM,
            )


# ##################### MAIN #####################

if __name__ == "__main__":
    try:
        rospy.init_node("controller")
        controller = Controller()
        controller.prepare_map()

        ani = FuncAnimation(
            plt.gcf(), controller.plot_states_etc, interval=50
        )  # pretty ugly since it makes us rely on the plotting to iterate it seems
        plt.show()

    except rospy.ROSInterruptException:
        pass
