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
    

    file_pattern = os.path.join("../Images/front_camera", '*.png')

    png_files = glob.glob(file_pattern)

    for image in png_files:
        template = cv2.imread(image)
        h, w, _ = template.shape

        cv_image_copy = cv_image.copy()

        if image is not None:
            
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

            #if image == "../Images/front_camera/bola_azul_front_back.png":
            #    print("Bola azul = " + str(min_val))
            #
            #if image == "../Images/front_camera/bola_roxa_front.png":
            #    print("Bola roxa = " + str(min_val))

            #if image == "../Images/front_camera/bola_vermelha_front.png":
            #    print("Bola vermelha = " + str(min_val))

            #if image == "../Images/front_camera/cubo_azul_front.png":
            #    print("Cubo azul = " + str(min_val))
            #
            #if image == "../Images/front_camera/person_back_front.png":
            #    print("Parte traseira da pessoa = " + str(min_val))

            #if image == "../Images/front_camera/pessoa_front.png":
            #    print("Parte frontal da pessoa = " + str(min_val))

            #if image == "../Images/front_camera/portatil_front.png":
            #    print("Portatil = " + str(min_val))

            #----------------------#
            #   Detection filter   #
            #----------------------#

            # detect gynastic ball
            if (image == "../Images/front_camera/bola_azul_front_back.png" and min_val >= 180000000.0 and min_val <= 200000000.0):
                cv2.putText(cv_image_copy, 'BOLA GINASIO', ((center_x - ((width // 2) // 2)), (center_y - (height // 2))), cv2.FONT_HERSHEY_SIMPLEX ,  1, (230,216,173), 2, cv2.LINE_AA) 
                cv2.rectangle(cv_image_copy, location, bottom_right, (230,216,173), 5)

            # detect dark orchid ball
            if (image == "../Images/front_camera/bola_roxa_front.png" and min_val >= 150000000.0 and min_val <= 180000000.0):
                cv2.putText(cv_image_copy, 'BOLA ROXA', ((center_x - ((width // 2) // 2)), (center_y - (height // 2))), cv2.FONT_HERSHEY_SIMPLEX ,  1, (204, 50, 153), 2, cv2.LINE_AA) 
                
                cv2.rectangle(cv_image_copy, location, bottom_right, (204, 50, 153), 5)

            # detect red ball
            if (image == "../Images/front_camera/bola_vermelha_front.png" and min_val >= 40000000.0 and min_val <= 90000000.0):
                cv2.putText(cv_image_copy, 'BOLA VERMELHA', ((center_x - ((width // 2) // 2)), (center_y - (height // 2))), cv2.FONT_HERSHEY_SIMPLEX ,  1, (0, 0, 255), 2, cv2.LINE_AA) 
                
                cv2.rectangle(cv_image_copy, location, bottom_right, (0, 0, 255), 5)

            # detect blue cube 
            if (image == "../Images/front_camera/cubo_azul_front.png" and min_val >= 10000000.0 and min_val <= 35000000.0):
                cv2.putText(cv_image_copy, 'CUBO AZUL', ((center_x - ((width // 2) // 2)), (center_y - (height // 2))), cv2.FONT_HERSHEY_SIMPLEX ,  1, (255,0,0), 2, cv2.LINE_AA) 
                
                cv2.rectangle(cv_image_copy, location, bottom_right, (255,0,0), 5)

            # detect frontal and back person
            if ((image == "../Images/front_camera/person_back_front.png" and min_val >= 60000000.0 and min_val <= 70000000.0)
                or (image == "../Images/front_camera/pessoa_front.png" and min_val >= 60000000.0 and min_val <= 70000000.0)):
                cv2.putText(cv_image_copy, 'PESSOA', ((center_x - ((width // 2) // 2)), (center_y - (height // 2))), cv2.FONT_HERSHEY_SIMPLEX ,  1, (0, 0, 0), 2, cv2.LINE_AA) 
                
                cv2.rectangle(cv_image_copy, location, bottom_right, (0, 0, 0), 5)

            # detect laptop
            if (image == "../Images/front_camera/portatil_front.png" and min_val >= 400000000.0 and min_val <= 800000000.0):
                cv2.putText(cv_image_copy, 'PORTATIL', ((center_x - ((width // 2) // 2)), (center_y - (height // 2))), cv2.FONT_HERSHEY_SIMPLEX ,  1, (0, 140, 255), 2, cv2.LINE_AA) 
                
                cv2.rectangle(cv_image_copy, location, bottom_right, (0, 140, 255), 5)

            cv2.imshow("Image", cv_image_copy)
            cv2.waitKey(1)

def main():
    topic = '/camera/rgb/image_raw'

    # Setup ROS
    rospy.init_node('image_converter', anonymous=True)
    rospy.Subscriber(topic, Image, detectObject)
    rospy.spin()

if __name__ == '__main__':
    main()