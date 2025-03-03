#!/usr/bin/env python3

import rospy
import math
import threading
from tf2_ros import Buffer, TransformListener, TransformException
import numpy as np
from autoware_mini.msg import Path, DetectedObjectArray, Waypoint
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3, Vector3Stamped
from shapely.geometry import LineString, Point, Polygon
from shapely import prepare, intersects, intersection
from tf2_geometry_msgs import do_transform_vector3
from scipy.interpolate import interp1d

import heapq


class SimpleLocalPlanner:

    def __init__(self):

        # Parameters
        self.output_frame = rospy.get_param("~output_frame")
        self.local_path_length = rospy.get_param("~local_path_length")
        self.transform_timeout = rospy.get_param("~transform_timeout")
        self.braking_safety_distance_obstacle = rospy.get_param(
            "~braking_safety_distance_obstacle"
        )
        self.braking_safety_distance_goal = rospy.get_param(
            "~braking_safety_distance_goal"
        )
        self.braking_reaction_time = rospy.get_param("braking_reaction_time")
        self.stopping_lateral_distance = rospy.get_param("stopping_lateral_distance")
        self.current_pose_to_car_front = rospy.get_param("current_pose_to_car_front")
        self.default_deceleration = rospy.get_param("default_deceleration")

        # Variables
        self.lock = threading.Lock()
        self.global_path_linestring = None
        self.global_path_distances = None
        self.distance_to_velocity_interpolator = None
        self.current_speed = None
        self.current_position = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)

        # Publishers
        self.local_path_pub = rospy.Publisher(
            "local_path", Path, queue_size=1, tcp_nodelay=True
        )

        # Subscribers
        rospy.Subscriber(
            "global_path", Path, self.path_callback, queue_size=None, tcp_nodelay=True
        )
        rospy.Subscriber(
            "/localization/current_pose",
            PoseStamped,
            self.current_pose_callback,
            queue_size=1,
            tcp_nodelay=True,
        )
        rospy.Subscriber(
            "/localization/current_velocity",
            TwistStamped,
            self.current_velocity_callback,
            queue_size=1,
            tcp_nodelay=True,
        )
        rospy.Subscriber(
            "/detection/final_objects",
            DetectedObjectArray,
            self.detected_objects_callback,
            queue_size=1,
            buff_size=2**20,
            tcp_nodelay=True,
        )

    def path_callback(self, msg):

        if len(msg.waypoints) == 0:
            global_path_linestring = None
            global_path_distances = None
            distance_to_velocity_interpolator = None
            rospy.loginfo("%s - Empty global path received", rospy.get_name())

        else:
            waypoints_xyz = np.array(
                [(w.position.x, w.position.y, w.position.z) for w in msg.waypoints]
            )
            # convert waypoints to shapely linestring
            global_path_linestring = LineString(waypoints_xyz)
            prepare(global_path_linestring)

            # calculate distances between points, use only xy, and insert 0 at start of distances array
            global_path_distances = np.cumsum(
                np.sqrt(np.sum(np.diff(waypoints_xyz[:, :2], axis=0) ** 2, axis=1))
            )
            global_path_distances = np.insert(global_path_distances, 0, 0)

            # extract velocity values at waypoints
            velocities = np.array([w.speed for w in msg.waypoints])
            # create interpolator
            distance_to_velocity_interpolator = interp1d(
                global_path_distances,
                velocities,
                kind="linear",
                bounds_error=False,
                fill_value=0.0,
            )

            rospy.loginfo(
                "%s - Global path received with %i waypoints",
                rospy.get_name(),
                len(msg.waypoints),
            )

        with self.lock:
            self.global_path_linestring = global_path_linestring
            self.global_path_distances = global_path_distances
            self.distance_to_velocity_interpolator = distance_to_velocity_interpolator

    def current_velocity_callback(self, msg):
        # save current velocity
        self.current_speed = msg.twist.linear.x

    def current_pose_callback(self, msg):
        # save current pose
        current_position = Point([msg.pose.position.x, msg.pose.position.y])
        self.current_position = current_position

    def detected_objects_callback(self, msg):
        print("------ detected objects callback, number of objects: ", len(msg.objects))

        # Check if all important class variables are set
        if any(
            var is None
            for var in [
                self.global_path_linestring,
                self.global_path_distances,
                self.distance_to_velocity_interpolator,
                self.current_speed,
                self.current_position,
            ]
        ):
            self.publish_local_path_wp([], msg.header.stamp, self.output_frame)
            return

        with self.lock:
            # Assign class variables to local variables
            global_path_linestring = self.global_path_linestring
            global_path_distances = self.global_path_distances
            distance_to_velocity_interpolator = self.distance_to_velocity_interpolator
            # current_speed = self.current_speed
            current_position = self.current_position

        d_ego_from_path_start = global_path_linestring.project(current_position)
        local_path = self.extract_local_path(
            global_path_linestring,
            global_path_distances,
            d_ego_from_path_start,
            self.local_path_length,
        )
        if local_path is None:
            self.publish_local_path_wp([], msg.header.stamp, self.output_frame)
            return

        map_based_target_velocity = distance_to_velocity_interpolator(
            d_ego_from_path_start
        )
        local_path_waypoints = self.convert_local_path_to_waypoints(
            local_path, map_based_target_velocity
        )
        if len(msg.objects) == 0:
            self.publish_local_path_wp(
                local_path_waypoints,
                msg.header.stamp,
                self.output_frame,
            )
        else:
            # create a buffer around the local path
            local_path_buffer = local_path.buffer(
                self.stopping_lateral_distance, cap_style="flat"
            )
            prepare(local_path_buffer)

            object_distances = []
            target_velocities = []
            object_velocities = []
            for detected_object in msg.objects:

                # We need to take the minimum distance for each object,
                # if it intersects with the local path.

                # Convert obj.convex_hull.points to a Shapely Polygon
                convex_hull_polygon = Polygon(
                    [(p.x, p.y) for p in detected_object.convex_hull.points]
                )
                if intersects(local_path_buffer, convex_hull_polygon):
                    # TODO Transform values seem to be weird, which causes the velocities not to change after transform
                    transform = self.tf_buffer.lookup_transform(
                        self.output_frame,
                        msg.header.frame_id,
                        msg.header.stamp,
                        rospy.Duration(self.transform_timeout),
                    )
                    if transform is not None:
                        print(transform)
                        vector3_stamped = Vector3Stamped(
                            vector=detected_object.velocity
                        )
                        vec = vector3_stamped.vector
                        #print("Velocity:", str(math.sqrt(vec.x**2 + vec.y**2 + vec.z**2)), end=" ;; ")
                        velocity = do_transform_vector3(vector3_stamped, transform)
                        vec = velocity.vector
                        #print(vector3_stamped.vector == vec)
                        #print("Transformed velocity:", str(math.sqrt(vec.x**2 + vec.y**2 + vec.z**2)))
                    else:
                        velocity = Vector3()
                    object_velocities.append(velocity)

                    distances = []
                    intersection_polygon = intersection(
                        local_path_buffer, convex_hull_polygon
                    )
                    for idx, point_coords in enumerate(
                        intersection_polygon.exterior.coords
                    ):
                        # First and last point are the same
                        if idx < len(intersection_polygon.exterior.coords) - 1:
                            d = local_path.project(Point(point_coords))

                            # heap has O(log n) complexity for push (list has O(1))
                            heapq.heappush(distances, d)
                    # BUT heap has O(1) complexity for finding the minimum element (list has O(n))
                    # Additionaly, consider current pose to car front distance and braking safety distance
                    distance_to_object = max(
                        0.0, distances[0] - self.current_pose_to_car_front
                    )
                    distance_to_object_with_safety = max(
                        0.0, distance_to_object - self.braking_safety_distance_obstacle
                    )
                    object_distances.append(distance_to_object)

                    object_velocity = 0.0  # Let it be 0 for now
                    # Calculate the target velocity for the object
                    target_velocity = math.sqrt(
                        max(
                            0,
                            object_velocity**2
                            + 2
                            * self.default_deceleration
                            * distance_to_object_with_safety,
                        )
                    )
                    target_velocities.append(target_velocity)

            # No objects detected, which intersect with the current local path
            # Which means we can publish the local path with no blocking objects
            if len(target_velocities) == 0:
                self.publish_local_path_wp(
                    local_path_waypoints,
                    msg.header.stamp,
                    self.output_frame,
                )
                return

            target_velocities = np.array(target_velocities)

            # print("target velocities: ", target_velocities)
            # print("object distances: ", object_distances)

            # Should be more optimal to use np.argmin instead of finding minimum element's index on regular python list
            # However, not completely sure if converting list to np.array overhead is worth it (its O(n))
            min_idx = np.argmin(target_velocities)

            # Always stay within the limit of the map-based target velocity
            min_target_velocity = min(
                target_velocities[min_idx], map_based_target_velocity
            )
            min_object_dist = object_distances[min_idx]

            # Recalculate target_velocity for all the waypoints using the closest object
            zero_speeds_onwards = False
            for i, wp in enumerate(local_path_waypoints):

                # once we get zero speed, keep it that way
                if zero_speeds_onwards:
                    wp.speed = 0.0
                    continue

                # overwrite target velocity of wp
                wp.speed = min(min_target_velocity, wp.speed)

                # from stop point onwards all speeds are set to zero
                if math.isclose(wp.speed, 0.0):
                    zero_speeds_onwards = True

            self.publish_local_path_wp(
                local_path_waypoints,
                msg.header.stamp,
                self.output_frame,
                closest_object_distance=min_object_dist,
                closest_object_velocity=0.0,  # for now
                local_path_blocked=True,
                stopping_point_distance=min_object_dist,
            )

    def extract_local_path(
        self,
        global_path_linestring,
        global_path_distances,
        d_ego_from_path_start,
        local_path_length,
    ):

        # current position is projected at the end of the global path - goal reached
        if math.isclose(d_ego_from_path_start, global_path_linestring.length):
            return None

        d_to_local_path_end = d_ego_from_path_start + local_path_length

        # find index where distances are higher than ego_d_on_global_path
        index_start = np.argmax(global_path_distances >= d_ego_from_path_start)
        index_end = np.argmax(global_path_distances >= d_to_local_path_end)

        # if end point of local_path is past the end of the global path (returns 0) then take index of last point
        if index_end == 0:
            index_end = len(global_path_linestring.coords) - 1

        # create local path from global path add interpolated points at start and end, use sliced point coordinates in between
        start_point = global_path_linestring.interpolate(d_ego_from_path_start)
        end_point = global_path_linestring.interpolate(d_to_local_path_end)
        local_path = LineString(
            [start_point]
            + list(global_path_linestring.coords[index_start:index_end])
            + [end_point]
        )

        return local_path

    def convert_local_path_to_waypoints(self, local_path, target_velocity):
        # convert local path to waypoints
        local_path_waypoints = []
        for point in local_path.coords:
            waypoint = Waypoint()
            waypoint.position.x = point[0]
            waypoint.position.y = point[1]
            waypoint.position.z = point[2]
            waypoint.speed = target_velocity
            local_path_waypoints.append(waypoint)
        return local_path_waypoints

    def publish_local_path_wp(
        self,
        local_path_waypoints,
        stamp,
        output_frame,
        closest_object_distance=0.0,
        closest_object_velocity=0.0,
        local_path_blocked=False,
        stopping_point_distance=0.0,
    ):
        # create path message
        path = Path()
        path.header.frame_id = output_frame
        path.header.stamp = stamp
        path.waypoints = local_path_waypoints
        path.closest_object_distance = closest_object_distance
        path.closest_object_velocity = closest_object_velocity
        path.is_blocked = local_path_blocked
        path.stopping_point_distance = stopping_point_distance  # there is no cost field in the Path message anymore
        self.local_path_pub.publish(path)

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    rospy.init_node("simple_local_planner")
    node = SimpleLocalPlanner()
    node.run()
