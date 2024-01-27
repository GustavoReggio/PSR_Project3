#!/usr/bin/env python3

import os
import glob
import numpy as np
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def detectObject(msg):

    bridge = CvBridge()

    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")

    file_pattern = os.path.join("/home/miguel/catkin_ws/src/git_grupo/PSR_Project3/project_yolo/Images/front_camera", '*.png')

    png_files = glob.glob(file_pattern)

    for image in png_files:
        template = cv2.imread(image)
        h, w, _ = template.shape

        if image is not None:
            
            result = cv2.matchTemplate(cv_image, template, cv2.TM_SQDIFF)
            min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)
            location = min_loc

            bottom_right = (location[0] + w, location[1] + h)
            cv2.rectangle(cv_image, location, bottom_right, 255, 5)

            cv2.imshow("Image", cv_image)
            cv2.waitKey(1)

def main():
    topic = '/camera/rgb/image_raw'

    # Setup ROS
    rospy.init_node('image_converter', anonymous=True)
    rospy.Subscriber(topic, Image, detectObject)
    rospy.spin()

if __name__ == '__main__':
    main()