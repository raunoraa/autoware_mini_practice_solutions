#!/usr/bin/env python3

import rospy
import numpy as np

from shapely import MultiPoint
from tf2_ros import TransformListener, Buffer, TransformException
from numpy.lib.recfunctions import structured_to_unstructured
from ros_numpy import numpify

from sensor_msgs.msg import PointCloud2
from autoware_mini.msg import DetectedObjectArray, DetectedObject
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point32

BLUE80P = ColorRGBA(0.0, 0.0, 1.0, 0.8)


class ClusterDetector:
    def __init__(self):
        self.min_cluster_size = rospy.get_param("~min_cluster_size")
        self.output_frame = rospy.get_param("/detection/output_frame")
        self.transform_timeout = rospy.get_param("~transform_timeout")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)

        self.objects_pub = rospy.Publisher(
            "detected_objects", DetectedObjectArray, queue_size=1, tcp_nodelay=True
        )
        rospy.Subscriber(
            "points_clustered",
            PointCloud2,
            self.cluster_callback,
            queue_size=1,
            buff_size=2**24,
            tcp_nodelay=True,
        )

        rospy.loginfo("%s - initialized", rospy.get_name())

    def cluster_callback(self, msg):
        data = numpify(msg)
        labels = data["label"]
        points = structured_to_unstructured(
            data[["x", "y", "z", "label"]], dtype=np.float32
        )

        if msg.header.frame_id != self.output_frame:
            # Fetch transform for target frame
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.output_frame,
                    msg.header.frame_id,
                    msg.header.stamp,
                    rospy.Duration(self.transform_timeout),
                )
            except (TransformException, rospy.ROSTimeMovedBackwardsException) as e:
                rospy.logwarn("%s - %s", rospy.get_name(), e)
                return

        tf_matrix = numpify(transform.transform).astype(np.float32)

        # turn into homogeneous coordinates
        points[:, 3] = 1
        # transform points to target frame
        points = points.dot(tf_matrix.T)
        dt_array = DetectedObjectArray()
        dt_array.header.stamp = msg.header.stamp
        dt_array.header.frame_id = self.output_frame

        detected_objects = []
        for label in np.unique(
            labels[labels != -1]
        ):  # Get unique valid labels directly
            mask = labels == label  # Create mask for current label

            # Get points corresponding to the label
            # Get only x, y, z coordinates
            points_cluster = points[mask, :3]
            if (
                points_cluster.shape[0] < self.min_cluster_size
            ):  # Check if the cluster is large enough
                continue

            # Create a detected object
            obj = DetectedObject()

            # Compute centroid
            centroid = np.mean(points_cluster, axis=0)
            obj.position = Point32(centroid[0], centroid[1], centroid[2])

            # Compute convex hull
            points_2d = MultiPoint(
                points_cluster[:, :2]
            )  # Use only x and y coordinates
            hull = points_2d.convex_hull
            # Convert hull polygon to list of Point32
            convex_hull_points = [
                Point32(x, y, centroid[2]) for x, y in hull.exterior.coords
            ]
            obj.convex_hull.points = convex_hull_points

            # Assign additional properties
            obj.id = label  # Use cluster label as ID
            obj.label = "unknown"
            obj.color = BLUE80P
            obj.valid = True
            obj.position_reliable = True
            obj.velocity_reliable = False
            obj.acceleration_reliable = False

            detected_objects.append(obj)

        dt_array.objects = detected_objects
        self.objects_pub.publish(dt_array)

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    rospy.init_node("cluster_detector", log_level=rospy.INFO)
    node = ClusterDetector()
    node.run()
