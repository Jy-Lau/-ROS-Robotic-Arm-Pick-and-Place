#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rospkg
import yaml

class Camera():

    def __init__(self):
        rospy.init_node('camera_node', anonymous=True)
        rospy.loginfo("Camera...")
        self._check_camera_ready()
        self.bridge = CvBridge()
        with open(rospkg.RosPack().get_path('robot_pnp') + f"/config/hsv_threshold.yaml", 'r') as f:
            hsv_yaml = yaml.safe_load(f)
        self.hsv_threshold = {}
        self.hsv_threshold['red'] = np.array(hsv_yaml['red'])
        self.hsv_threshold['green'] = np.array(hsv_yaml['green'])
        self.hsv_threshold['blue'] = np.array(hsv_yaml['blue'])
        self.map_color={}
        self.map_color[0] = 'Red'
        self.map_color[1] = 'Green'
        self.map_color[2] = 'Blue'
        self.camera_subscriber = rospy.Subscriber('/camera/depth/image_raw', Image, self.camera_callback)

    def _check_camera_ready(self):
        camera_msg = None
        rospy.loginfo("Checking Laser...")
        while camera_msg is None and not rospy.is_shutdown():
            try:
                camera_msg = rospy.wait_for_message("/camera/depth/image_raw", Image, timeout=1.0)
                rospy.logdebug("Current /camera/image_raw READY=>" + str(camera_msg))

            except:
                rospy.logdebug("Current /camera/depth/image_raw not ready yet, retrying for getting camera")
        rospy.loginfo("Checking Camera...DONE")

    def camera_callback(self, msg):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")
            rgb_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # Display image in OpenCV window
            cv2.imshow("Camera Feed", rgb_image)
            cv2.waitKey(1)
            color = self._find_color(rgb_image)
            # rospy.loginfo(f"{self.map_color[color]} cube detected!")
        except CvBridgeError as e:
            rospy.logerr(e)
            return
            

    def _find_color(self, img):
        kernel = np.ones((5, 5), np.float32)/25
        blur = cv2.filter2D(img, -1, kernel)
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        
        mask_red = cv2.inRange(hsv, self.hsv_threshold['red'][0], self.hsv_threshold['red'][1])
        mask_green = cv2.inRange(hsv, self.hsv_threshold['green'][0], self.hsv_threshold['green'][1])
        mask_blue = cv2.inRange(hsv, self.hsv_threshold['blue'][0], self.hsv_threshold['blue'][1])
        
        rgb_mask_count = [cv2.countNonZero(mask_red),cv2.countNonZero(mask_green),cv2.countNonZero(mask_blue)]
        index = rgb_mask_count.index(max(rgb_mask_count)) #0 is red, 1 is green, 2 is blue
        return index

if __name__ == '__main__':
    camera = Camera()
    rospy.spin()