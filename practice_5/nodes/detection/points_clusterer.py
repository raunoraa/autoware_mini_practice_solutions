#!/usr/bin/env python3

import rospy
import ros_numpy
import numpy as np

from sensor_msgs.msg import PointCloud2
from numpy.lib.recfunctions import (
    structured_to_unstructured,
    unstructured_to_structured,
)
from sklearn.cluster import DBSCAN


class PointsClusterer:
    def __init__(self):

        # Parameters
        self.cluster_epsilon = rospy.get_param("~cluster_epsilon")
        self.cluster_min_size = rospy.get_param("~cluster_min_size")

        self.clusterer = DBSCAN(
            eps=self.cluster_epsilon, min_samples=self.cluster_min_size
        )

        # Publishers
        self.clustered_points_pub = rospy.Publisher(
            "points_clustered",
            PointCloud2,
            queue_size=1,
            tcp_nodelay=True,
        )

        # Subscribers
        rospy.Subscriber(
            "points_filtered",
            PointCloud2,
            self.points_callback,
            queue_size=1,
            buff_size=2**24,
            tcp_nodelay=True,
        )

    def points_callback(self, msg):
        data = ros_numpy.numpify(msg)
        points = structured_to_unstructured(data[["x", "y", "z"]], dtype=np.float32)
        point_labels = self.clusterer.fit_predict(points)
        assert points.shape[0] == point_labels.shape[0]
        
        # Merge points and label arrays along axis 1. The label array must be expanded to 2D to ensure matching dimensions
        points_labeled = np.concatenate((points, point_labels[:, np.newaxis]), axis=1)

        # Filter out labels with value -1
        points_labeled = points_labeled[points_labeled[:,3] != -1]      
        data = unstructured_to_structured(points_labeled, dtype=np.dtype([
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('label', np.int32)
        ]))
                
        cluster_msg = ros_numpy.msgify(PointCloud2, data)
        cluster_msg.header.frame_id = msg.header.frame_id
        cluster_msg.header.stamp = msg.header.stamp
        
        self.clustered_points_pub.publish(cluster_msg)
        
        
    def run(self):
        rospy.spin()


if __name__ == "__main__":
    rospy.init_node("points_clusterer")
    node = PointsClusterer()
    node.run()
