#!/usr/bin/env python3

import numpy as np
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def detectObject(msg):

    bridge = CvBridge()

    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")

    templateBallGym = cv2.imread("../Images/front_camera/bola_azul_front_back.png")
    templateBallViolet = cv2.imread("../Images/front_camera/bola_roxa_front.png")
    templateBallRed = cv2.imread("../Images/front_camera/bola_vermelha_front.png")
    templateCubeBlue = cv2.imread("../Images/front_camera/cubo_azul_front.png")
    templatePersonBack = cv2.imread("../Images/front_camera/person_back_front.png")
    templatePersonFront = cv2.imread("../Images/front_camera/pessoa_front.png")
    templateLaptop = cv2.imread("../Images/front_camera/portatil_front.png")

    hBallGym, wBallGym, _ = templateBallGym.shape
    hBallViolet, wBallViolet, _ = templateBallViolet.shape
    hBallRed, wBallRed, _ = templateBallRed.shape
    hCubeBlue, wCubeBlue, _ = templateCubeBlue.shape
    hPersonBack, wPersonBack, _ = templatePersonBack.shape
    hPersonFront, wPersonFront, _ = templatePersonFront.shape
    hLaptop, wLaptop, _ = templateLaptop.shape

    cv_image_copy = cv_image.copy()

    cv_imageGray = cv2.cvtColor(cv_image_copy, cv2.COLOR_BGR2GRAY)

    resultBallGym = cv2.matchTemplate(cv_image_copy, templateBallGym, cv2.TM_SQDIFF)
    resultBallViolet = cv2.matchTemplate(cv_image_copy, templateBallViolet, cv2.TM_SQDIFF)
    resultBallRed = cv2.matchTemplate(cv_image_copy, templateBallRed, cv2.TM_SQDIFF)
    resultCubeBlue = cv2.matchTemplate(cv_image_copy, templateCubeBlue, cv2.TM_SQDIFF)
    resultPersonBack = cv2.matchTemplate(cv_image_copy, templatePersonBack, cv2.TM_SQDIFF)
    resultPersonFront = cv2.matchTemplate(cv_image_copy, templatePersonFront, cv2.TM_SQDIFF)
    resultLaptop = cv2.matchTemplate(cv_image_copy, templateLaptop, cv2.TM_SQDIFF)

    min_val_BallGym, _, min_loc_BallGym, _ = cv2.minMaxLoc(resultBallGym)
    min_val_BallViolet, _, min_loc_BallViolet, _ = cv2.minMaxLoc(resultBallViolet)
    min_val_BallRed, _, min_loc_BallRed, _ = cv2.minMaxLoc(resultBallRed)
    min_val_CubeBlue, _, min_loc_CubeBlue, _ = cv2.minMaxLoc(resultCubeBlue)
    min_val_PersonBack, _, min_loc_PersonBack, _ = cv2.minMaxLoc(resultPersonBack)
    min_val_PersonFront, _, min_loc_PersonFront, _ = cv2.minMaxLoc(resultPersonFront)
    min_val_Laptop, _, min_loc_Laptop, _ = cv2.minMaxLoc(resultLaptop)

    locationBallGym = min_loc_BallGym
    bottom_right_BallGym = (locationBallGym[0] + wBallGym, locationBallGym[1] + hBallGym)

    locationBallViolet = min_loc_BallViolet
    bottom_right_BallViolet = (locationBallViolet[0] + wBallViolet, locationBallViolet[1] + hBallViolet)

    locationBallRed = min_loc_BallRed
    bottom_right_BallRed = (locationBallRed[0] + wBallRed, locationBallRed[1] + hBallRed)

    locationCubeBlue = min_loc_CubeBlue
    bottom_right_CubeBlue = (locationCubeBlue[0] + wCubeBlue, locationCubeBlue[1] + hCubeBlue)

    locationPersonBack = min_loc_PersonBack
    bottom_right_PersonBack = (locationPersonBack[0] + wPersonBack, locationPersonBack[1] + hPersonBack)

    locationPersonFront = min_loc_PersonFront
    bottom_right_PersonFront = (locationPersonFront[0] + wPersonFront, locationPersonFront[1] + hPersonFront)

    locationLaptop = min_loc_Laptop
    bottom_right_Laptop = (locationLaptop[0] + wLaptop, locationLaptop[1] + hLaptop)

    # get center
    center_x_BallGym = (locationBallGym[0] + bottom_right_BallGym[0]) // 2
    center_y_BallGym = (locationBallGym[1] + bottom_right_BallGym[1]) // 2

    # get height and width
    heightBallGym = bottom_right_BallGym[1] - locationBallGym[1]
    widthBallGym = bottom_right_BallGym[0] - locationBallGym[1]

    # get center
    center_x_BallViolet = (locationBallViolet[0] + bottom_right_BallViolet[0]) // 2
    center_y_BallViolet = (locationBallViolet[1] + bottom_right_BallViolet[1]) // 2

    # get height and width
    heightBallViolet = bottom_right_BallViolet[1] - locationBallViolet[1]
    widthBallViolet = bottom_right_BallViolet[0] - locationBallViolet[1]

    # get center
    center_x_BallRed = (locationBallRed[0] + bottom_right_BallRed[0]) // 2
    center_y_BallRed = (locationBallRed[1] + bottom_right_BallRed[1]) // 2

    # get height and width
    heightBallRed = bottom_right_BallRed[1] - locationBallRed[1]
    widthBallRed = bottom_right_BallRed[0] - locationBallRed[1]

    # get center
    center_x_CubeBlue = (locationCubeBlue[0] + bottom_right_CubeBlue[0]) // 2
    center_y_CubeBlue = (locationCubeBlue[1] + bottom_right_CubeBlue[1]) // 2

    # get height and width
    heightCubeBlue = bottom_right_CubeBlue[1] - locationCubeBlue[1]
    widthCubeBlue = bottom_right_CubeBlue[0] - locationCubeBlue[1]

    # get center
    center_x_PersonBack = (locationPersonBack[0] + bottom_right_PersonBack[0]) // 2
    center_y_PersonBack = (locationPersonBack[1] + bottom_right_PersonBack[1]) // 2

    # get height and width
    heightPersonBack = bottom_right_PersonBack[1] - locationPersonBack[1]
    widthPersonBack = bottom_right_PersonBack[0] - locationPersonBack[1]

    # get center
    center_x_PersonFront = (locationPersonFront[0] + bottom_right_PersonFront[0]) // 2
    center_y_PersonFront = (locationPersonFront[1] + bottom_right_PersonFront[1]) // 2

    # get height and width
    heightPersonFront = bottom_right_PersonFront[1] - locationPersonFront[1]
    widthPersonFront = bottom_right_PersonFront[0] - locationPersonFront[1]

    # get center
    center_x_Laptop = (locationLaptop[0] + bottom_right_Laptop[0]) // 2
    center_y_Laptop = (locationLaptop[1] + bottom_right_Laptop[1]) // 2

    # get height and width
    heightLaptop = bottom_right_Laptop[1] - locationLaptop[1]
    widthLaptop = bottom_right_Laptop[0] - locationLaptop[1]

    #-----------#
    #   Debug   #
    #-----------#

    #if image == "../Images/front_camera/bola_azul_front_back.png":
    #print("Bola azul = " + str(min_val_BallGym))
    #
    #if image == "../Images/front_camera/bola_roxa_front.png":
    #print("Bola roxa = " + str(min_val_BallViolet))

    #if image == "../Images/front_camera/bola_vermelha_front.png":
    #print("Bola vermelha = " + str(min_val_BallRed))

    #if image == "../Images/front_camera/cubo_azul_front.png":
    #print("Cubo azul = " + str(min_val_CubeBlue))
    #
    #if image == "../Images/front_camera/person_back_front.png":
    print("Parte traseira da pessoa = " + str(min_val_PersonBack))

    #if image == "../Images/front_camera/pessoa_front.png":
    print("Parte frontal da pessoa = " + str(min_val_PersonFront))

    #if image == "../Images/front_camera/portatil_front.png":
    #print("Portatil = " + str(min_val_Laptop))

    #----------------------#
    #   Detection filter   #
    #----------------------#

    # detect gynastic ball
    if (min_val_BallGym >= 180000000.0 and min_val_BallGym <= 200000000.0):
        cv2.putText(cv_image_copy, 'BOLA GINASIO', ((center_x_BallGym - ((widthBallGym // 2) // 2)), (center_y_BallGym - (heightBallGym // 2))), cv2.FONT_HERSHEY_SIMPLEX ,  1, (230,216,173), 2, cv2.LINE_AA) 
        cv2.rectangle(cv_image_copy, locationBallGym, bottom_right_BallGym, (230,216,173), 5)

    # detect dark orchid ball
    if (min_val_BallViolet >= 150000000.0 and min_val_BallViolet <= 180000000.0):
        cv2.putText(cv_image_copy, 'BOLA ROXA', ((center_x_BallViolet - ((widthBallViolet // 2) // 2)), (center_y_BallViolet - (heightBallViolet // 2))), cv2.FONT_HERSHEY_SIMPLEX ,  1, (204, 50, 153), 2, cv2.LINE_AA) 
        cv2.rectangle(cv_image_copy, locationBallViolet, bottom_right_BallViolet, (204, 50, 153), 5)

    # detect red ball
    if (min_val_BallRed >= 40000000.0 and min_val_BallRed <= 90000000.0):
        cv2.putText(cv_image_copy, 'BOLA VERMELHA', ((center_x_BallRed - ((widthBallRed // 2) // 2)), (center_y_BallRed - (heightBallRed // 2))), cv2.FONT_HERSHEY_SIMPLEX ,  1, (0, 0, 255), 2, cv2.LINE_AA) 
        cv2.rectangle(cv_image_copy, locationBallRed, bottom_right_BallRed, (0, 0, 255), 5)

    # detect blue cube 
    if (min_val_CubeBlue >= 10000000.0 and min_val_CubeBlue <= 35000000.0):
        cv2.putText(cv_image_copy, 'CUBO AZUL', ((center_x_CubeBlue - ((widthCubeBlue // 2) // 2)), (center_y_CubeBlue - (heightCubeBlue // 2))), cv2.FONT_HERSHEY_SIMPLEX ,  1, (255,0,0), 2, cv2.LINE_AA) 
        cv2.rectangle(cv_image_copy, locationCubeBlue, bottom_right_CubeBlue, (255,0,0), 5)

    # detect frontal and back person
    if (min_val_PersonBack >= 60000000.0 and min_val_PersonBack <= 70000000.0):
        cv2.putText(cv_image_copy, 'PESSOA', ((center_x_PersonBack - ((widthPersonBack // 2) // 2)), (center_y_PersonBack - (heightPersonBack // 2))), cv2.FONT_HERSHEY_SIMPLEX ,  1, (0, 0, 0), 2, cv2.LINE_AA) 
        cv2.rectangle(cv_image_copy, locationPersonBack, bottom_right_PersonBack, (0, 0, 0), 5)

    if (min_val_PersonFront >=60000000.0 and min_val_PersonFront <= 70000000.0):
        cv2.putText(cv_image_copy, 'PESSOA', ((center_x_PersonFront - ((widthPersonFront // 2) // 2)), (center_y_PersonFront - (heightPersonFront // 2))), cv2.FONT_HERSHEY_SIMPLEX ,  1, (0, 0, 0), 2, cv2.LINE_AA) 
        cv2.rectangle(cv_image_copy, locationPersonFront, bottom_right_PersonFront, (0, 0, 0), 5)

    # detect laptop
    if (min_val_Laptop >= 400000000.0 and min_val_Laptop <= 800000000.0):
        cv2.putText(cv_image_copy, 'PORTATIL', ((center_x_Laptop - ((widthLaptop // 2) // 2)), (center_y_Laptop - (heightLaptop // 2))), cv2.FONT_HERSHEY_SIMPLEX ,  1, (0, 140, 255), 2, cv2.LINE_AA) 
        cv2.rectangle(cv_image_copy, locationLaptop, bottom_right_Laptop, (0, 140, 255), 5)

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