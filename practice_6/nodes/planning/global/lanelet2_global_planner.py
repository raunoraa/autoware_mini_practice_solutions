#!/usr/bin/env python3

import rospy
import math
import shapely

from geometry_msgs.msg import PoseStamped
from autoware_mini.msg import Path, Waypoint

# Lanelet 2 imports
import lanelet2
from lanelet2.io import Origin, load
from lanelet2.projection import UtmProjector
from lanelet2.core import BasicPoint2d
from lanelet2.geometry import findNearest


# Helper function for calculating the distance between two points
def calculate_dist(point1, point2):
    return math.sqrt((point2.x - point1.x) ** 2 + (point2.y - point1.y) ** 2)


class Lanelet2GlobalPlanner:
    def __init__(self):

        # Parameters
        self.default_speed_limit = rospy.get_param("~speed_limit")
        self.output_frame = rospy.get_param("lanelet2_global_planner/output_frame")

        self.distance_to_goal_limit = rospy.get_param(
            "lanelet2_global_planner/distance_to_goal_limit"
        )

        # Parameters for lanelet2 map loading:
        # lanelet2_map_name: name of the lanelet2 map file
        # coordinate_transformer: coordinate transformer
        # use_custom_origin: use custom origin
        # utm_origin_lat: utm origin latitude
        # utm_origin_lon: utm origin longitude
        self.lanelet2_map_name = rospy.get_param("~lanelet2_map_name")
        self.coordinate_transformer = rospy.get_param(
            "/localization/coordinate_transformer"
        )
        self.use_custom_origin = rospy.get_param("/localization/use_custom_origin")
        self.utm_origin_lat = rospy.get_param("/localization/utm_origin_lat")
        self.utm_origin_lon = rospy.get_param("/localization/utm_origin_lon")

        self.lanelet2_loaded_map = self.load_lanelet2_map()

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

    def load_lanelet2_map(self):
        """
        Load a lanelet2 map from a file and return it
        :return: lanelet2 map
        """

        # Load the map using Lanelet2
        if self.coordinate_transformer == "utm":
            projector = UtmProjector(
                Origin(self.utm_origin_lat, self.utm_origin_lon),
                self.use_custom_origin,
                False,
            )
        else:
            raise ValueError(
                'Unknown coordinate_transformer for loading the Lanelet2 map ("utm" should be used): '
                + self.coordinate_transformer
            )

        lanelet2_map = load(self.lanelet2_map_name, projector)

        return lanelet2_map

    def is_speed_valid(self, speed_str):
        # Checks if the given speed is valid:
        # 1) speed string can be converted to float;
        # 2) speed is not negative;
        # 3) speed is less or equal to the default speed limit.

        try:
            speed = float(speed_str)
            if speed >= 0 and speed <= self.default_speed_limit:
                return True
        except ValueError:
            pass  # If conversion to float fails, it's invalid
        return False

    def convert_lanelet_sequence_to_waypoints(
        self, lanelet_sequence, user_selected_goalpoint
    ):

        # returns waypoints seqeunce and sets the goal point on path

        waypoints = []
        for l_idx, lanelet in enumerate(lanelet_sequence):

            speed = None
            # code to check if lanelet has attribute speed_ref and if the speed is valid
            if "speed_ref" in lanelet.attributes and self.is_speed_valid(
                lanelet.attributes["speed_ref"]
            ):
                speed = float(lanelet.attributes["speed_ref"])
            else:
                speed = self.default_speed_limit
            # convert speed from km/h to m/s
            speed /= 3.6

            # if we are looking at the last lanelet, there will be the goalpoint
            goal_point_on_path = None
            lanelet_path = None
            dist_on_path = None
            if l_idx == len(lanelet_sequence) - 1:
                # Create a Shapely LineString for the centerline of the last lanelet
                lanelet_path = shapely.LineString(
                    [(point.x, point.y) for point in lanelet.centerline]
                )
                # Project the goal point onto the centerline of the last lanelet
                dist_on_path = lanelet_path.project(
                    shapely.Point(user_selected_goalpoint.x, user_selected_goalpoint.y)
                )
                goal_point_on_path = lanelet_path.interpolate(dist_on_path)

                self.goal_point = BasicPoint2d(
                    goal_point_on_path.x, goal_point_on_path.y
                )

            for p_idx, point in enumerate(lanelet.centerline):

                # for avoiding overlapping points of start and end, omit the last waypoint of the lanelet
                # except for the last lanelet, because no lanelet will follow that
                if l_idx < len(lanelet_sequence) - 1:
                    if p_idx == len(lanelet.centerline) - 1:
                        break
                else:
                    # discard points that are after the goal point on the path
                    if (
                        lanelet_path.project(shapely.Point(point.x, point.y))
                        >= dist_on_path
                    ):
                        # create goal waypoint, get x, y and z from the goal point on the path
                        waypoint = Waypoint()
                        waypoint.position.x = self.goal_point.x
                        waypoint.position.y = self.goal_point.y
                        waypoint.position.z = goal_point_on_path.z

                        # For speed, just take the current point's values
                        waypoint.speed = speed
                        waypoints.append(waypoint)
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

    def find_path_no_lane_change(self, current_location, goal_point):

        # get start and end lanelets
        start_lanelet = findNearest(
            self.lanelet2_loaded_map.laneletLayer, current_location, 1
        )[0][1]
        goal_lanelet = findNearest(
            self.lanelet2_loaded_map.laneletLayer, goal_point, 1
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
        return path_no_lane_change

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

        # Using the following idea to sync the user selected goalpoint and the corresponding waypoint:
        # Create a new waypoint - the closest point on the path to the user-selected goal point and use that as the path end and goal point.

        current_location_now = self.current_location
        user_selected_goalpoint = BasicPoint2d(msg.pose.position.x, msg.pose.position.y)
        path_no_lane_change = self.find_path_no_lane_change(
            current_location_now, user_selected_goalpoint
        )

        waypoints_from_seq = self.convert_lanelet_sequence_to_waypoints(
            path_no_lane_change, user_selected_goalpoint
        )

        self.publish_global_path(waypoints_from_seq)

    def current_pose_callback(self, msg):
        self.current_location = BasicPoint2d(msg.pose.position.x, msg.pose.position.y)

        if self.goal_point is None:
            return

        dist = calculate_dist(self.current_location, self.goal_point)
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
