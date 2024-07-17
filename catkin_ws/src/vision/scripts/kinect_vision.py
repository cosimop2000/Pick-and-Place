#!/usr/bin/env python3

import cv2 as cv
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

# Initialize ROS node
rospy.init_node('vision', anonymous=True)

# Initialize CvBridge
bridge = CvBridge()

# Flag to show images
a_show = True

def process_image(rgb, depth):
    print(rgb)
    print(depth)
    if rgb is not None:
        # Display and save the RGB image
        if a_show:
            cv.imshow("RGB Image", rgb)
            cv.imwrite("rgb.png", rgb)
        cv.waitKey(1)
    
    if depth is not None:
        # Convert depth image to a displayable format
        depth_display = cv.normalize(depth, None, 0, 255, cv.NORM_MINMAX)
        depth_display = np.uint8(depth_display)
        
        # Display and save the depth image
        if a_show:
            cv.imshow("Depth Image", depth_display)
            cv.imwrite("depth.png", depth_display)
        cv.waitKey(1)

def rgb_callback(data):
    rgb_image = bridge.imgmsg_to_cv2(data, 'bgr8')
    process_image(rgb_image, None)

def depth_callback(data):
    depth_image = bridge.imgmsg_to_cv2(data, '32FC1')
    process_image(None, depth_image)

# Set up subscribers
rgb_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, rgb_callback)
depth_sub = rospy.Subscriber('/camera/depth/image_raw', Image, depth_callback)

# Keep the node running
rospy.spin()
