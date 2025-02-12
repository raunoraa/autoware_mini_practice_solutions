#!/usr/bin/env python3

import rospy

from geometry_msgs.msg import PoseStamped


# Lanelet 2 imports
import lanelet2
from lanelet2.io import Origin, load
from lanelet2.projection import UtmProjector
from lanelet2.core import BasicPoint2d
from lanelet2.geometry import findNearest


class Lanelet2GlobalPlanner:
    def __init__(self):

        # Parameters
        
        lanelet2_map_name = rospy.get_param("~lanelet2_map_name")
        self.lanelet2_loaded_map = self.load_lanelet2_map(lanelet2_map_name)        
        
        # traffic rules
        traffic_rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                          lanelet2.traffic_rules.Participants.VehicleTaxi)
        # routing graph
        self.graph = lanelet2.routing.RoutingGraph(self.lanelet2_loaded_map, traffic_rules)
        
        
        self.current_location = None
        self.goal_point = None        

        # Publishers
        

        # Subscribers
        rospy.Subscriber(
            "/move_base_simple/goal",
            PoseStamped,
            self.goalpoint_callback,
            queue_size=1
        )
        
        rospy.Subscriber(
            "/localization/current_pose",
            PoseStamped,
            self.current_pose_callback,
            queue_size=1
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
            projector = UtmProjector(Origin(utm_origin_lat, utm_origin_lon), use_custom_origin, False)
        else:
            raise ValueError('Unknown coordinate_transformer for loading the Lanelet2 map ("utm" should be used): ' + coordinate_transformer)

        lanelet2_map = load(lanelet2_map_name, projector)

        return lanelet2_map
        
    def goalpoint_callback(self, msg):
        # loginfo message about receiving the goal point
        rospy.loginfo(
            "%s - goal position (%f, %f, %f) orientation (%f, %f, %f, %f) in %s frame", rospy.get_name(),
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
            msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z,
            msg.pose.orientation.w, msg.header.frame_id
        )
        
        self.goal_point = BasicPoint2d(msg.pose.position.x, msg.pose.position.y)
        
        if self.current_location:
            # get start and end lanelets
            start_lanelet = findNearest(self.lanelet2_loaded_map.laneletLayer, self.current_location, 1)[0][1]
            goal_lanelet = findNearest(self.lanelet2_loaded_map.laneletLayer, self.goal_point, 1)[0][1]
            # find routing graph
            route = self.graph.getRoute(start_lanelet, goal_lanelet, 0, True)

            if not route:
                rospy.logwarn("Impossible to reach the given goal!")
                return None
            
            # find shortest path
            path = route.shortestPath()
            
            # this returns LaneletSequence to a point where lane change would be necessary to continue
            path_no_lane_change = path.getRemainingLane(start_lanelet)
            print(path_no_lane_change)
            
        else:
            rospy.logwarn(
                "Current location has not been initialized yet!"
            )
        
    def current_pose_callback(self, msg):
        self.current_location = BasicPoint2d(msg.pose.position.x, msg.pose.position.y)

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    rospy.init_node("lanelet2_global_planner")
    node = Lanelet2GlobalPlanner()
    node.run()
