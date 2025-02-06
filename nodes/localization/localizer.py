#!/usr/bin/env python3

import math
import rospy

from tf.transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster
from pyproj import CRS, Transformer, Proj

from novatel_oem7_msgs.msg import INSPVA
from geometry_msgs.msg import PoseStamped, TwistStamped, Quaternion, TransformStamped

class Localizer:
    def __init__(self):

        # Parameters
        self.undulation = rospy.get_param('undulation')
        utm_origin_lat = rospy.get_param('utm_origin_lat')
        utm_origin_lon = rospy.get_param('utm_origin_lon')

        # Internal variables
        self.crs_wgs84 = CRS.from_epsg(4326)
        self.crs_utm = CRS.from_epsg(25835)
        self.utm_projection = Proj(self.crs_utm)
        
        # Create coordinate transformer
        self.transformer = Transformer.from_crs(self.crs_wgs84, self.crs_utm)
        self.origin_x, self.origin_y = self.transformer.transform(utm_origin_lat, utm_origin_lon)

        # Subscribers
        rospy.Subscriber('/novatel/oem7/inspva', INSPVA, self.transform_coordinates)

        # Publishers
        self.current_pose_pub = rospy.Publisher('current_pose', PoseStamped, queue_size=10)
        self.current_velocity_pub = rospy.Publisher('current_velocity', TwistStamped, queue_size=10)
        self.br = TransformBroadcaster()
        

    def transform_coordinates(self, msg):
        # print(msg.latitude, msg.longitude)
        
        current_x, current_y = self.transformer.transform(msg.latitude, msg.longitude)
        current_transform_x = current_x - self.origin_x
        current_transform_y = current_y - self.origin_y
        print(current_transform_x, current_transform_y)
        
        # calculate azimuth correction
        azimuth_correction = self.utm_projection.get_factors(msg.longitude, msg.latitude).meridian_convergence
        corrected_azimuth = msg.azimuth - azimuth_correction
        
        # convert azimuth to yaw angle
        def convert_azimuth_to_yaw(azimuth):
            """
            Converts azimuth to yaw. Azimuth is CW angle from the North. Yaw is CCW angle from the East.
            :param azimuth: azimuth in radians
            :return: yaw in radians
            """
            yaw = -azimuth + math.pi/2
            # Clamp within 0 to 2 pi
            if yaw > 2 * math.pi:
                yaw = yaw - 2 * math.pi
            elif yaw < 0:
                yaw += 2 * math.pi

            return yaw

        yaw = convert_azimuth_to_yaw(corrected_azimuth)
        
        # Convert yaw to quaternion
        x, y, z, w = quaternion_from_euler(0, 0, yaw)
        orientation = Quaternion(x, y, z, w)
        
        # publish current pose
        current_pose_msg = PoseStamped()
        current_pose_msg.header.stamp = msg.header.stamp
        current_pose_msg.header.frame_id = "map"
        current_pose_msg.pose.position.x = current_transform_x
        current_pose_msg.pose.position.y = current_transform_y
        current_pose_msg.pose.position.z = msg.height - self.undulation
        current_pose_msg.pose.orientation = orientation
        self.current_pose_pub.publish(current_pose_msg)
        

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('localizer')
    node = Localizer()
    node.run()