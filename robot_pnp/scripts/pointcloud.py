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
from geometry_msgs.msg import PointStamped, Pose
from robot_pnp.srv import PointArray
from geometry_msgs.msg import Point
import threading 
class PointCloud():

    def __init__(self):
        rospy.init_node('point_cloud_node', anonymous=True)
        rospy.loginfo("Point Cloud...")
        self._check_pointcloud_ready()
        self.lock = threading.Lock()
        self.point_cloud_subscriber = rospy.Subscriber('/camera/depth/points', PointCloud2, self.point_cloud_callback)
        camera_subscriber = rospy.Subscriber('/camera/contour_coodinates', Point, self.camera_callback)
        rospy.loginfo("Waiting for robot_pnp service...")
        rospy.wait_for_service('robot_pnp')  # wait for service to be available
        rospy.loginfo("Found service robot_pnp")
        try:
            # create service proxy
            self.pnp_service = rospy.ServiceProxy('robot_pnp', PointArray)
        except rospy.ServiceException as e:
            # handle exception
            rospy.logerr(e)

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
        # Define search radius and iteration pattern
        search_radius = 3
        self.search_pattern = []  # Start with the centroid pixel
        for r in range(search_radius):
            # Generate spiral or concentric circle search pattern
            for i in range(r):
                self.search_pattern.append((i, r))
                self.search_pattern.append((-i, -r))
                self.search_pattern.append((r, -i))
                self.search_pattern.append((-r, i))

    def point_cloud_callback(self, point_cloud):
        self.frame_id = point_cloud.header.frame_id
        # point_step = point_cloud.point_step  # 32 bytes
        # endian_format = '>' if point_cloud.is_bigendian else '<'
        # num_points = len(point_cloud.data) // point_step

        # x = np.zeros((num_points,), dtype=np.float32)  # Replace with your actual blue channel values
        # y = np.zeros((num_points,), dtype=np.float32)  # Replace with your actual blue channel values
        # z = np.zeros((num_points,), dtype=np.float32)  # Replace with your actual blue channel values

        # for i in range(num_points):
        #     point_data = point_cloud.data[i * point_step: (i + 1) * point_step]
        #     x[i], y[i], z[i], rgb_value = struct.unpack(f'{endian_format}ffff', point_data[:12] + point_data[16:20])  # Assuming little-endian byte order         
        # self._get_rgb_from_pointcloud((red,green,blue),point_cloud.height,point_cloud.width)
        # self.point_cloud_array = np.stack((x, y, z), axis=-1)
        # self.point_cloud_array = self.point_cloud_array.reshape((point_cloud.height, point_cloud.width, 3))
        with self.lock:
            point_cloud_array2_list =list(point_cloud2.read_points(point_cloud,field_names = ('x', 'y', 'z'), skip_nans=False))
            point_cloud_array = np.array(point_cloud_array2_list, dtype=np.float32)
            point_cloud_array = point_cloud_array.reshape((point_cloud.height, point_cloud.width, 3))
            self.point_cloud_array = point_cloud_array
        
    def camera_callback(self, msg):
        print('Received callback from camera')
        y_pixel=int(msg.y)
        x_pixel=int(msg.x)
        x_coord=0
        y_coord=0
        z_coord=0
        if np.isnan(self.point_cloud_array[y_pixel][x_pixel]).any():
            # Define search radius and iteration pattern
            for dx, dy in self.search_pattern:
                y_pixel = y_pixel + dy
                x_pixel = x_pixel + dx
                print(f'Pixel Y: {y_pixel}, dx: {dx}')
                print(f'Pixel X: {x_pixel}, dy: {dy}')
                # Check if the point cloud data is valid (not NaN)
                if not np.isnan(self.point_cloud_array[y_pixel][x_pixel]).any():
                    # Extract the x, y, and z coordinates
                    x_coord = self.point_cloud_array[y_pixel][x_pixel][0]
                    y_coord = self.point_cloud_array[y_pixel][x_pixel][1]
                    z_coord = self.point_cloud_array[y_pixel][x_pixel][2]
                    break
        else:
            x_coord=self.point_cloud_array[y_pixel][x_pixel][0]
            y_coord=self.point_cloud_array[y_pixel][x_pixel][1]
            z_coord=self.point_cloud_array[y_pixel][x_pixel][2]

        print(f'Final Camera x: {x_coord}')
        print(f'Final Camera y: {y_coord}')
        print(f'Final Camera z: {z_coord}')
        print(f'Final y_pixel: {y_pixel}')
        print(f'Final x_pixel: {x_pixel}')

        if x_coord!=0 and y_coord!=0 and z_coord!=0:
            camera_coordinates = PointStamped()
            camera_coordinates.header.frame_id = self.frame_id
            camera_coordinates.point.x = x_coord
            camera_coordinates.point.y = y_coord
            camera_coordinates.point.z = z_coord
             # Wait for the transformation to become available
            listener = tf.TransformListener()
            listener.waitForTransform('world', self.frame_id, rospy.Time(), rospy.Duration(1.0))
            try:
                world_coordinates = listener.transformPoint('world', camera_coordinates)
                print(f'Final World x: {world_coordinates.point.x}')
                print(f'Final World y: {world_coordinates.point.y}')
                print(f'Final World z: {world_coordinates.point.z}')
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logerr("Failed to transform camera coordinates to end effector frame.")
            response = self.pnp_service([world_coordinates])
            print('Response from server: ', response.status)
            self.point_cloud_subscriber.unregister()


            
if __name__ == '__main__':
    pointcloud = PointCloud()
    rospy.spin()