#!/usr/bin/env python3

import rospy
import numpy as np

from autoware_mini.msg import Path, VehicleCmd
from geometry_msgs.msg import PoseStamped

from shapely.geometry import LineString, Point
from shapely import prepare, distance

from tf.transformations import euler_from_quaternion

from scipy.interpolate import interp1d


class PurePursuitFollower:
    def __init__(self):

        # Parameters
        self.lookahead_distance = rospy.get_param("~lookahead_distance")
        self.wheel_base = rospy.get_param("/vehicle/wheel_base")

        self.path_var = None

        self.distance_to_velocity_interpolator = None

        # Publishers
        self.vehicle_cmd_pub = rospy.Publisher("vehicle_cmd", VehicleCmd, queue_size=10)

        # Subscribers
        rospy.Subscriber("path", Path, self.path_callback, queue_size=1)
        rospy.Subscriber(
            "/localization/current_pose",
            PoseStamped,
            self.current_pose_callback,
            queue_size=1,
        )

    def path_callback(self, msg):

        # convert waypoints to shapely linestring
        path_linestring = LineString(
            [(w.position.x, w.position.y) for w in msg.waypoints]
        )
        # prepare path - creates spatial tree, making the spatial queries more efficient
        prepare(path_linestring)

        self.path_var = path_linestring

        # Create a distance to velocity interpolator for the path
        # collect waypoint x and y coordinates
        waypoints_xy = np.array([(w.position.x, w.position.y) for w in msg.waypoints])
        # Calculate distances between points
        distances = np.cumsum(
            np.sqrt(np.sum(np.diff(waypoints_xy, axis=0) ** 2, axis=1))
        )
        # add 0 distance in the beginning
        distances = np.insert(distances, 0, 0)
        # Extract velocity values at waypoints
        velocities = np.array([w.speed for w in msg.waypoints])
        self.distance_to_velocity_interpolator = interp1d(
            distances, velocities, kind="linear"
        )

    def current_pose_callback(self, msg):
        vehicle_cmd = VehicleCmd()
        vehicle_cmd.header.stamp = msg.header.stamp
        vehicle_cmd.header.frame_id = "base_link"

        current_pose = Point([msg.pose.position.x, msg.pose.position.y])
        if self.path_var:
            d_ego_from_path_start = self.path_var.project(current_pose)
            _, _, heading = euler_from_quaternion(
                [
                    msg.pose.orientation.x,
                    msg.pose.orientation.y,
                    msg.pose.orientation.z,
                    msg.pose.orientation.w,
                ]
            )

            lookahead_point = self.path_var.interpolate(
                d_ego_from_path_start + self.lookahead_distance
            )
            lookahead_heading = np.arctan2(
                lookahead_point.y - current_pose.y, lookahead_point.x - current_pose.x
            )

            l_d = distance(current_pose, lookahead_point)
            alpha = lookahead_heading - heading

            vehicle_cmd.ctrl_cmd.steering_angle = np.arctan(
                (2 * self.wheel_base * np.sin(alpha)) / l_d
            )

            if self.distance_to_velocity_interpolator:
                vehicle_cmd.ctrl_cmd.linear_velocity = (
                    self.distance_to_velocity_interpolator(d_ego_from_path_start)
                )

        self.vehicle_cmd_pub.publish(vehicle_cmd)

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    rospy.init_node("pure_pursuit_follower")
    node = PurePursuitFollower()
    node.run()

"""
Vastus küsimusele:
Suurema lookahead distance puhul hakkab masin kurve lõikama, 
sest suurema lookahead distance puhul on järgmine waypoint tee peal alles peale kurvi.
Sõiduk üritab võimalikult otse liikuda järgmise waypointini, mistõttu ei arvesta sõiduk vahepealse reaalse teega.
"""