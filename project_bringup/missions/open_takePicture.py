#!/usr/bin/env python3

import numpy as np
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import mission_manager

# counter to keep track of the number imaged saved from the moment of initialization of the node
check_imaged_saved_count = 0

def takePictureFront(msg):
    global check_imaged_saved_count

    bridge = CvBridge()

    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")

    cv_image_copy = cv_image.copy()

    fileName = "../images_Saved/front_camera.png"

    cv2.imwrite(fileName, cv_image_copy)

    check_imaged_saved_count = check_imaged_saved_count + 1

    check_shutdown_condition()

def takePictureBack(msg):
    global check_imaged_saved_count

    bridge = CvBridge()

    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")

    cv_image_copy = cv_image.copy()

    fileName = "../images_Saved/back_camera.png"

    cv2.imwrite(fileName, cv_image_copy)

    check_imaged_saved_count = check_imaged_saved_count + 1

    check_shutdown_condition()

def takePictureTop(msg):
    global check_imaged_saved_count

    bridge = CvBridge()

    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")

    cv_image_copy = cv_image.copy()

    fileName = "../images_Saved/top_camera.png"

    cv2.imwrite(fileName, cv_image_copy)

    check_imaged_saved_count = check_imaged_saved_count + 1

    check_shutdown_condition()

def check_shutdown_condition():
    global check_imaged_saved_count

    # check if it saved all images
    if check_imaged_saved_count == 3:
        print("All images saved. Shutting down the node.")
        rospy.signal_shutdown("All images saved. Node shutdown.")

def main():

    while(1):
        print(mission_manager.result_msg)

        if mission_manager.result_msg is not None:
            topic_front = '/camera/rgb/image_raw'
            topic_back = '/camera/rgb/image_raw_back'
            topic_top = '/camera/rgb/image_raw_up'

            rospy.init_node('take_picture', anonymous=True)

            # subscribe topic of cameras to receive images
            rospy.Subscriber(topic_front, Image, takePictureFront, queue_size=1)
            rospy.Subscriber(topic_back, Image, takePictureBack, queue_size=1)
            rospy.Subscriber(topic_top, Image, takePictureTop, queue_size=1)

            rospy.spin()


if __name__ == '__main__':
    main()