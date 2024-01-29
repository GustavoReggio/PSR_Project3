#!/usr/bin/env python3

#---------------#
#   Libraries   #
#---------------#

import os
import glob
import numpy as np
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

#---------------------------------#
#   Function to detect object     #
#---------------------------------#

def detectEmptyTable(msg):
    bridge = CvBridge()

    # get image from topic
    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    
    # search images
    file_pattern = os.path.join("../Images/top_camera_mesa", '*.png')

    png_files = glob.glob(file_pattern)

    # go throught all images
    for image in png_files:
        template = cv2.imread(image)
        h, w, _ = template.shape

        cv_image_copy = cv_image.copy()

        if image is not None:
            
            # match stream with template
            result = cv2.matchTemplate(cv_image_copy, template, cv2.TM_SQDIFF)
            min_val, _, min_loc, _ = cv2.minMaxLoc(result)
            location = min_loc
            bottom_right = (location[0] + w, location[1] + h)

            # get center
            center_x = (location[0] + bottom_right[0]) // 2
            center_y = (location[1] + bottom_right[1]) // 2

            # get height and width
            height = bottom_right[1] - location[1]
            width = bottom_right[0] - location[1]

            #-----------#
            #   Debug   #
            #-----------#

            #if image == "../Images/top_camera/mesa_livre.png":
            #    print("Mesa vazia = " + str(min_val))
            #if image == "../Images/top_camera/mesa_livre_cozinha.png":
            #    print("Mesa vazia = " + str(min_val))

            #----------------------#
            #   Detection filter   #
            #----------------------#

            # detect free table
            if (image == "../Images/top_camera_mesa/mesa_livre.png" and min_val >= 60000000.0 and min_val <= 150000000.0):
                cv2.putText(cv_image_copy, 'MESA LIVRE', ((center_x - ((width // 2) // 2)), (center_y - (height // 2))), cv2.FONT_HERSHEY_SIMPLEX ,  1, (0,255,0), 2, cv2.LINE_AA) 
                cv2.rectangle(cv_image_copy, location, bottom_right, (0,255,0), 5)
            # detect table ocupy
            elif (image == "../Images/top_camera_mesa/mesa_livre.png" and min_val > 150000000.0 and min_val <= 400000000.0):
                cv2.putText(cv_image_copy, 'MESA OCUPADA', ((center_x - ((width // 2) // 2)), (center_y - (height // 2))), cv2.FONT_HERSHEY_SIMPLEX ,  1, (0,0,255), 2, cv2.LINE_AA) 
                cv2.rectangle(cv_image_copy, location, bottom_right, (0,0,255), 5)

            # detect free table
            if (image == "../Images/top_camera_mesa/mesa_livre_cozinha.png" and min_val >= 400000000.0 and min_val <= 500000000.0):
                cv2.putText(cv_image_copy, 'MESA LIVRE', ((center_x - ((width // 2) // 2)), (center_y - (height // 2))), cv2.FONT_HERSHEY_SIMPLEX ,  1, (0,255,0), 2, cv2.LINE_AA) 
                cv2.rectangle(cv_image_copy, location, bottom_right, (0,255,0), 5)
            # detect table ocupy
            elif ((image == "../Images/top_camera_mesa/mesa_livre_cozinha.png" and min_val > 50000000.0 and min_val < 400000000.0)
                  or (image == "../Images/top_camera/mesa_livre_cozinha.png" and min_val > 500000000.0 and min_val < 700000000.0)):
                cv2.putText(cv_image_copy, 'MESA OCUPADA', ((center_x - ((width // 2) // 2)), (center_y - (height // 2))), cv2.FONT_HERSHEY_SIMPLEX ,  1, (0,0,255), 2, cv2.LINE_AA) 
                cv2.rectangle(cv_image_copy, location, bottom_right, (0,0,255), 5)

            cv2.imshow("Image", cv_image_copy)
            key = cv2.waitKey(1)

            #-----------------#
            #   Termination   #
            #-----------------#

            if key == ord('w'):
                print('Saving image')
                # TODO how to save canvas?
                cv2.imwrite('../saved_Images/photo_top.png', cv_image)
            elif key == ord('q'):
                print("Exiting!!")
                rospy.signal_shutdown("Exiting!!. Node shutdown.")

def main():

    #--------------------#
    #   Initialization   #
    #--------------------#

    topic_top = '/camera/rgb/image_raw_up'


    #---------------#
    #   Execution   #
    #---------------#

    # Setup ROS
    rospy.init_node('image_converter', anonymous=True)
    rospy.Subscriber(topic_top, Image, detectEmptyTable)
    rospy.spin()

if __name__ == '__main__':
    main()