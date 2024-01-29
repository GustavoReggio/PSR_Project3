#!/usr/bin/env python3

from functools import partial
import numpy
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *

from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped

from tf.transformations import quaternion_from_euler
from move_base_msgs.msg import MoveBaseActionResult

server = None
marker_pos = 0.8

menu_handler = MenuHandler()

h_first_entry = 0
h_mode_last = 0

# counter to keep track of the number imaged saved from the moment of initialization of the node
check_imaged_saved_count = 0

def enableCb(feedback):
    handle = feedback.menu_entry_id
    state = menu_handler.getCheckState(handle)

    if state == MenuHandler.CHECKED:
        menu_handler.setCheckState(handle, MenuHandler.UNCHECKED)
        rospy.loginfo("Hiding first menu entry")
        menu_handler.setVisible(h_first_entry, False)
    else:
        menu_handler.setCheckState(handle, MenuHandler.CHECKED)
        rospy.loginfo("Showing first menu entry")
        menu_handler.setVisible(h_first_entry, True)

    menu_handler.reApply(server)
    rospy.loginfo("update")
    server.applyChanges()


def modeCb(feedback):
    global h_mode_last
    menu_handler.setCheckState(h_mode_last, MenuHandler.UNCHECKED)
    h_mode_last = feedback.menu_entry_id
    menu_handler.setCheckState(h_mode_last, MenuHandler.CHECKED)

    rospy.loginfo("Switching to menu entry #" + str(h_mode_last))
    menu_handler.reApply(server)
    print("DONE")
    server.applyChanges()


def makeBox(msg):
    marker = Marker()

    marker.type = Marker.CYLINDER
    marker.scale.x = msg.scale * 0.45
    marker.scale.y = msg.scale * 0.45
    marker.scale.z = msg.scale * 0.45
    marker.color.r = 0.5
    marker.color.g = 0.5
    marker.color.b = 0.5
    marker.color.a = 0.2

    return marker


def makeBoxControl(msg):
    control = InteractiveMarkerControl()
    control.always_visible = True
    control.markers.append(makeBox(msg))
    msg.controls.append(control)
    return control


def makeEmptyMarker(dummyBox=True):
    global marker_pos
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "base_link"
    int_marker.pose.position.z = marker_pos
    marker_pos += 1
    int_marker.scale = 1
    return int_marker


def makeMenuMarker(name):
    int_marker = makeEmptyMarker()
    int_marker.name = name

    control = InteractiveMarkerControl()

    control.interaction_mode = InteractiveMarkerControl.BUTTON
    control.always_visible = True

    control.markers.append(makeBox(int_marker))
    int_marker.controls.append(control)

    server.insert(int_marker)


def deepCb(feedback):
    rospy.loginfo("The deep sub-menu has been found.")

def moveTo(feedback, x, y, z, R, P, Y, location, goal_publisher):
    
    result_msg = None

    print('Called moving to ' + location)
    p = Pose()
    p.position = Point(x=x, y=y, z=z)
    q = quaternion_from_euler(R, P, Y)  # From euler angles (rpy) to quaternion
    p.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

    ps = PoseStamped()
    ps.pose = p
    ps.header = Header(frame_id='map', stamp=rospy.Time.now())

    goal = 'Sending Goal move to ' + location

    print(goal)

    goal_publisher.publish(ps)

    # TODO know when move is finished

    try:
        result_msg = rospy.wait_for_message('/move_base/result', MoveBaseActionResult, timeout=60)
    except:
        print('Timeout waiting for moveto')
        # TODO
        return

def moveToCapture(feedback, x, y, z, R, P, Y, location, goal_publisher):

    bridge = CvBridge()

    result_msg = None

    print('Called moving to ' + location)
    p = Pose()
    p.position = Point(x=x, y=y, z=z)
    q = quaternion_from_euler(R, P, Y)  # From euler angles (rpy) to quaternion
    p.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

    ps = PoseStamped()
    ps.pose = p
    ps.header = Header(frame_id='map', stamp=rospy.Time.now())

    goal = 'Sending Goal move to ' + location

    print(goal)
    goal_publisher.publish(ps)

    try:
        result_msg = rospy.wait_for_message('/move_base/result', MoveBaseActionResult, timeout=60)
    except:
        print('Timeout waiting for move_base result')
        # TODO handle the timeout
        return

    print('move base completed goal with result ' + str(result_msg))

    try:

        result_msg_front = rospy.wait_for_message('/camera/rgb/image_raw', Image, timeout=10) 
        result_msg_back = rospy.wait_for_message('/camera/rgb/image_raw_back', Image, timeout=10) 
        result_msg_top = rospy.wait_for_message('/camera/rgb/image_raw_up', Image, timeout=10)

        cv_image_front = bridge.imgmsg_to_cv2(result_msg_front, "bgr8")
        cv_image_back = bridge.imgmsg_to_cv2(result_msg_back, "bgr8")
        cv_image_top = bridge.imgmsg_to_cv2(result_msg_top, "bgr8")

        cv_image_front_copy = cv_image_front.copy()
        cv_image_back_copy = cv_image_back.copy()
        cv_image_top_copy = cv_image_top.copy()

        cv2.imwrite("front_camera_" + location + ".png", cv_image_front_copy)
        cv2.imwrite("back_camera_" + location + ".png", cv_image_back_copy)
        cv2.imwrite("top_camera_" + location + ".png", cv_image_top_copy)

    except rospy.exceptions.ROSException:
        print('Timeout waiting for images')
        # TODO handle the timeout for image capture
        return

def main():

    global server

    # ---------------#
    # Initialization #
    # ---------------#

    rospy.init_node("mission_manager")

    # create move_base_simple/goal publisher
    goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)

    server = InteractiveMarkerServer("mission")
    print(server)

    global h_first_entry, h_mode_last
    h_first_entry = menu_handler.insert("Move to")

    entry = menu_handler.insert("Kitchen", parent=h_first_entry,
                                callback=partial(moveTo,
                                                 x=6.568593, y=-1.788789, z=0,
                                                 R=0, P=0, Y=-1.504141,
                                                 location='Kitchen',
                                                 goal_publisher=goal_publisher))
    
    entry = menu_handler.insert("Dinner Table", parent=h_first_entry,
                                callback=partial(moveTo,
                                                 x=4.826607, y=0.897721, z=-0.001008,
                                                 R=-0.000003, P=0.003184, Y=0.023003,
                                                 location='Dinner Table',
                                                 goal_publisher=goal_publisher))

    entry = menu_handler.insert("Bedroom", parent=h_first_entry,
                                callback=partial(moveTo,
                                                 x=-4.409525, y=-0.182006, z=0,
                                                 R=-0.000007, P=0.003198, Y=1.980398,
                                                 location='Bedroom',
                                                 goal_publisher=goal_publisher))
    
    entry = menu_handler.insert("Gym", parent=h_first_entry,
                                callback=partial(moveTo,
                                                 x=2.146100, y=2.415270, z=0,
                                                 R=-0.000007, P=0.003198, Y=1.980398,
                                                 location='Gym',
                                                 goal_publisher=goal_publisher))
    
    entry = menu_handler.insert("LivingRoom", parent=h_first_entry,
                                callback=partial(moveTo,
                                                x=1.407235, y=-0.186747, z=-0.001010,
                                                R=-0.000008, P=0.003199, Y=-1.559985,
                                                 location='LivingRoom',
                                                 goal_publisher=goal_publisher))
    
    entry = menu_handler.insert("BackRoom", parent=h_first_entry,
                                callback=partial(moveTo,
                                                 x=-5.260976, y=-3.623521, z=0,
                                                 R=-0.000007, P=0.003198, Y=1.980398,
                                                 location='BackRoom',
                                                 goal_publisher=goal_publisher))

    h_first_entry_sec = menu_handler.insert("Capture")

    entry = menu_handler.insert("Kitchen", parent=h_first_entry_sec,
                                callback=partial(moveToCapture,
                                                 x=6.568593, y=-1.788789, z=0,
                                                 R=0, P=0, Y=-1.504141,
                                                 location='Kitchen',
                                                 goal_publisher=goal_publisher))
    
    entry = menu_handler.insert("Dinner Table", parent=h_first_entry_sec,
                                callback=partial(moveToCapture,
                                                 x=4.826607, y=0.897721, z=-0.001008,
                                                 R=-0.000003, P=0.003184, Y=0.023003,
                                                 location='Dinner Table',
                                                 goal_publisher=goal_publisher))

    entry = menu_handler.insert("Bedroom", parent=h_first_entry_sec,
                                callback=partial(moveToCapture,
                                                 x=-4.409525, y=-0.182006, z=0,
                                                 R=-0.000007, P=0.003198, Y=1.980398,
                                                 location='Bedroom',
                                                 goal_publisher=goal_publisher))
    
    entry = menu_handler.insert("Gym", parent=h_first_entry_sec,
                                callback=partial(moveToCapture,
                                                 x=2.146100, y=2.415270, z=0,
                                                 R=-0.000007, P=0.003198, Y=1.980398,
                                                 location='Gym',
                                                 goal_publisher=goal_publisher))
    
    entry = menu_handler.insert("LivingRoom", parent=h_first_entry_sec,
                                callback=partial(moveToCapture,
                                                x=1.407235, y=-0.186747, z=-0.001010,
                                                R=-0.000008, P=0.003199, Y=-1.559985,
                                                 location='LivingRoom',
                                                 goal_publisher=goal_publisher))
    
    entry = menu_handler.insert("BackRoom", parent=h_first_entry_sec,
                                callback=partial(moveToCapture,
                                                 x=-5.260976, y=-3.623521, z=0,
                                                 R=-0.000007, P=0.003198, Y=1.980398,
                                                 location='BackRoom',
                                                 goal_publisher=goal_publisher))

    h_first_entry_third = menu_handler.insert("See Table")
    
    entry = menu_handler.insert("Dinner Table", parent=h_first_entry_third,
                                callback=partial(moveTo,
                                                 x=4.826607, y=0.897721, z=-0.001008,
                                                 R=-0.000003, P=0.003184, Y=0.023003,
                                                 location='Dinner Table',
                                                 goal_publisher=goal_publisher))

    entry = menu_handler.insert("LivingRoom", parent=h_first_entry_third,
                            callback=partial(moveTo,
                                                x=1.407235, y=-0.186747, z=-0.001010,
                                                R=-0.000008, P=0.003199, Y=-1.559985,
                                                location='LivingRoom',
                                                goal_publisher=goal_publisher))

    makeMenuMarker("marker1")

    menu_handler.apply(server, "marker1")
    server.applyChanges()

    rospy.spin()


if __name__ == '__main__':
    main()