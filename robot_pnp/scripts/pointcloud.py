#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import struct
import tf

class PointCloud():

    def __init__(self):
        rospy.init_node('point_cloud_node', anonymous=True)
        rospy.loginfo("Point Cloud...")
        self._check_pointcloud_ready()
        self.point_cloud_subscriber = rospy.Subscriber('/camera/depth/points', PointCloud2, self.point_cloud_callback)

    def _check_pointcloud_ready(self):
        pointcloud_msg = None
        rospy.loginfo("Checking Point Cloud...")
        while pointcloud_msg is None and not rospy.is_shutdown():
            try:
                pointcloud_msg = rospy.wait_for_message("/camera/depth/points", PointCloud2, timeout=1.0)
                rospy.logdebug("Current /camera/depth/pointsREADY=>" + str(pointcloud_msg))

            except:
                rospy.logdebug("Current /camera/depth/points not ready yet, retrying for getting camera")
        rospy.loginfo("Checking Point Cloud...DONE")

    def point_cloud_callback(self, point_cloud):
        point_step = point_cloud.point_step  # 32 bytes
        endian_format = '>' if point_cloud.is_bigendian else '<'
        num_points = len(point_cloud.data) // point_step

        x = np.zeros((num_points,), dtype=np.float32)  # Replace with your actual blue channel values
        y = np.zeros((num_points,), dtype=np.float32)  # Replace with your actual blue channel values
        z = np.zeros((num_points,), dtype=np.float32)  # Replace with your actual blue channel values

        for i in range(num_points):
            point_data = point_cloud.data[i * point_step: (i + 1) * point_step]
            x[i], y[i], z[i], rgb_value = struct.unpack(f'{endian_format}ffff', point_data[:12] + point_data[16:20])  # Assuming little-endian byte order         
        # self._get_rgb_from_pointcloud((red,green,blue),point_cloud.height,point_cloud.width)
        point_cloud_array = np.stack((x, y, z), axis=-1)
        point_cloud_array = point_cloud_array.reshape((point_cloud.width, point_cloud.height, 3))
        #point_cloud_array =list(point_cloud2.read_points(point_cloud,field_names = ('x', 'y', 'z','rgb'), skip_nans=False)) --Alternative way

        if np.isnan(point_cloud_array[538, 725]).any():
            # Define search radius and iteration pattern
            search_radius = 5
            search_pattern = []  # Start with the centroid pixel

            for r in range(search_radius):
                # Generate spiral or concentric circle search pattern
                for i in range(r):
                    search_pattern.append((i, r))
                    search_pattern.append((-i, -r))
                    search_pattern.append((r, -i))
                    search_pattern.append((-r, i))

                # Iterate over the search pattern
            for dx, dy in search_pattern:
                x = 538 + dx
                y = 725 + dy
                # Check if the point cloud data is valid (not NaN)
                if not np.isnan(point_cloud_array[x][y]).any():
                    # Extract the x, y, and z coordinates
                    x_coord = point_cloud_array[x][y][0]
                    y_coord = point_cloud_array[x][y][1]
                    z_coord = point_cloud_array[x][y][2]
                    print(f'x: {x_coord}')
                    print(f'y: {y_coord}')
                    print(f'z: {z_coord}')
                    print(f'x + dx:{x}')
                    print(f'x + dx:{y}')
                    break






if __name__ == '__main__':
    pointcloud = PointCloud()
    rospy.spin()