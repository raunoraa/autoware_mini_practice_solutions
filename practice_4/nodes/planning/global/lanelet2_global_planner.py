#!/usr/bin/env python3

import rospy
import math

from geometry_msgs.msg import PoseStamped
from autoware_mini.msg import Path, Waypoint

# Lanelet 2 imports
import lanelet2
from lanelet2.io import Origin, load
from lanelet2.projection import UtmProjector
from lanelet2.core import BasicPoint2d
from lanelet2.geometry import findNearest


class Lanelet2GlobalPlanner:
    def __init__(self):

        # Parameters
        self.default_speed_limit = rospy.get_param("~speed_limit")
        self.output_frame = rospy.get_param("lanelet2_global_planner/output_frame")

        lanelet2_map_name = rospy.get_param("~lanelet2_map_name")
        self.lanelet2_loaded_map = self.load_lanelet2_map(lanelet2_map_name)

        self.distance_to_goal_limit = rospy.get_param(
            "lanelet2_global_planner/distance_to_goal_limit"
        )

        # traffic rules
        traffic_rules = lanelet2.traffic_rules.create(
            lanelet2.traffic_rules.Locations.Germany,
            lanelet2.traffic_rules.Participants.VehicleTaxi,
        )
        # routing graph
        self.graph = lanelet2.routing.RoutingGraph(
            self.lanelet2_loaded_map, traffic_rules
        )

        self.current_location = None
        self.goal_point = None

        # Publishers
        self.global_path_pub = rospy.Publisher(
            "/planning/global_path", Path, queue_size=10
        )

        # Subscribers
        rospy.Subscriber(
            "/move_base_simple/goal", PoseStamped, self.goalpoint_callback, queue_size=1
        )

        rospy.Subscriber(
            "/localization/current_pose",
            PoseStamped,
            self.current_pose_callback,
            queue_size=1,
        )

    def load_lanelet2_map(self, lanelet2_map_name):
        """
        Load a lanelet2 map from a file and return it
        :param lanelet2_map_name: name of the lanelet2 map file
        :param coordinate_transformer: coordinate transformer
        :param use_custom_origin: use custom origin
        :param utm_origin_lat: utm origin latitude
        :param utm_origin_lon: utm origin longitude
        :return: lanelet2 map
        """

        # get parameters
        coordinate_transformer = rospy.get_param("/localization/coordinate_transformer")
        use_custom_origin = rospy.get_param("/localization/use_custom_origin")
        utm_origin_lat = rospy.get_param("/localization/utm_origin_lat")
        utm_origin_lon = rospy.get_param("/localization/utm_origin_lon")

        # Load the map using Lanelet2
        if coordinate_transformer == "utm":
            projector = UtmProjector(
                Origin(utm_origin_lat, utm_origin_lon), use_custom_origin, False
            )
        else:
            raise ValueError(
                'Unknown coordinate_transformer for loading the Lanelet2 map ("utm" should be used): '
                + coordinate_transformer
            )

        lanelet2_map = load(lanelet2_map_name, projector)

        return lanelet2_map

    def convert_lanelet_sequence_to_waypoints(self, lanelet_sequence):

        waypoints = []
        for l_idx, lanelet in enumerate(lanelet_sequence):

            speed = None
            # code to check if lanelet has attribute speed_ref
            if "speed_ref" in lanelet.attributes:
                speed = float(lanelet.attributes["speed_ref"])
            else:
                speed = self.default_speed_limit
            # convert speed from km/h to m/s
            speed = (speed * 1000) / 3600
            for p_idx, point in enumerate(lanelet.centerline):

                # for avoiding overlapping points of start and end, omit the last waypoint of the lanelet
                # except for the last lanelet, because no lanelet will follow that
                if l_idx < len(lanelet_sequence) - 1:
                    if p_idx == len(lanelet.centerline) - 1:
                        break

                # create Waypoint and get the coordinates from lanelet.centerline points
                waypoint = Waypoint()
                waypoint.position.x = point.x
                waypoint.position.y = point.y
                waypoint.position.z = point.z
                waypoint.speed = speed
                waypoints.append(waypoint)

        return waypoints

    def publish_global_path(self, waypoints):
        g_path = Path()
        g_path.header.frame_id = self.output_frame
        g_path.header.stamp = rospy.Time.now()
        g_path.waypoints = waypoints
        self.global_path_pub.publish(g_path)

    def calculate_dist(self, point1, point2):
        return math.sqrt((point2.x - point1.x) ** 2 + (point2.y - point1.y) ** 2)

    def goalpoint_callback(self, msg):

        if self.current_location is None:
            rospy.logwarn("Current location has not been initialized yet!")
            return

        # loginfo message about receiving the goal point
        rospy.loginfo(
            "%s - goal position (%f, %f, %f) orientation (%f, %f, %f, %f) in %s frame",
            rospy.get_name(),
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
            msg.header.frame_id,
        )

        self.goal_point = BasicPoint2d(msg.pose.position.x, msg.pose.position.y)

        # get start and end lanelets
        start_lanelet = findNearest(
            self.lanelet2_loaded_map.laneletLayer, self.current_location, 1
        )[0][1]
        goal_lanelet = findNearest(
            self.lanelet2_loaded_map.laneletLayer, self.goal_point, 1
        )[0][1]
        # find routing graph
        route = self.graph.getRoute(start_lanelet, goal_lanelet, 0, True)

        if route is None:
            rospy.logwarn("Impossible to reach the given goal!")
            return

        # find shortest path
        path = route.shortestPath()

        # this returns LaneletSequence to a point where lane change would be necessary to continue
        path_no_lane_change = path.getRemainingLane(start_lanelet)

        # reset the goal point to be in the same position as the last lanelet of the path
        last_lanelet = path_no_lane_change[-1]
        last_waypoint = last_lanelet.centerline[-1]
        self.goal_point = BasicPoint2d(last_waypoint.x, last_waypoint.y)

        waypoints_from_seq = self.convert_lanelet_sequence_to_waypoints(
            path_no_lane_change
        )
        self.publish_global_path(waypoints_from_seq)

    def current_pose_callback(self, msg):
        self.current_location = BasicPoint2d(msg.pose.position.x, msg.pose.position.y)

        if self.goal_point is None:
            return

        dist = self.calculate_dist(self.current_location, self.goal_point)
        if dist <= self.distance_to_goal_limit:
            rospy.loginfo("Destination reached!")
            self.publish_global_path([])  # Publish an empty path
            self.goal_point = None
            return

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    rospy.init_node("lanelet2_global_planner")
    node = Lanelet2GlobalPlanner()
    node.run()
