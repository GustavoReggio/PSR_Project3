#!/usr/bin/env python3

import random

import rospy
import rospkg
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler
import uuid
import argparse


def main():

    # -------------------------------
    # Initialization
    # -------------------------------
    parser = argparse.ArgumentParser(description='Script to compute perfect numbers.')
    parser.add_argument('-l', '--location', type=str, help='', required=False,
                        default='on_bed')
    parser.add_argument('-o', '--object', type=str, help='', required=False,
                        default='cube_b')
    
    parser.add_argument('-m', '--menu', type=str, nargs='?', help='Mostar o Menu de objetos e Locais disponíveis', required=False,
                        const=True)

    args = vars(parser.parse_args())  # creates a dictionary
    print(args)

    rospack = rospkg.RosPack()
    package_path = rospack.get_path('project_description') + '/models/'



            ####________Posições______####
    
    poses = {}
    q = quaternion_from_euler(0, 0, 0) 
    q13 = quaternion_from_euler(0, 0, 3.14) 
    # on bed pose
    p1 = Pose()
    p1.position = Point(x=-6.033466, y=1.971232, z=0.644345)
    # From euler angles (rpy) to quaternion
    p1.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    poses['on_bed'] = {'pose': p1}

    # on bed-side-table-right pose
    p2 = Pose()
    p2.position = Point(x=-4.489786, y=2.867268, z=0.679033) 
    # From euler angles (rpy) to quaternion
    p2.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    poses['on_bed_side_table_right'] = {'pose': p2}

    # on bed-side-table-left pose
    p3 = Pose()
    p3.position = Point(x=-7.703492, y=2.832678, z=0.7474) 
    # From euler angles (rpy) to quaternion
    p3.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    poses['on_bed_side_table_left'] = {'pose': p3}

    # on bedroom-table pose
    p4 = Pose()
    p4.position = Point(x=-8.920621, y=1.971230, z=0.759914) 
    # From euler angles (rpy) to quaternion
    p4.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    poses['on_bedroom_table'] = {'pose': p4}


    # On shelf
    p6 = Pose()
    p6.position = Point(x=4.321983, y=-5.099168, z=0.383039)
    p6.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    poses['on_shelf'] = {'pose': p6}

    # on bedroom floor pose
    p7 = Pose()
    p7.position = Point(x=-4.153343, y=-4.627160, z=0.318684) 
    # From euler angles (rpy) to quaternion
    p7.orientation = Quaternion(x=q13[0], y=q13[1], z=q13[2], w=q13[3])
    poses['on_bedroom_flor'] = {'pose': p7}

    # On sofa
    p8 = Pose()
    p8.position = Point(x=-0.231274, y=-1.361250, z=0.500937)
    # From euler angles (rpy) to quaternion
    p8.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    poses['on_sofa'] = {'pose': p8}

    # On_top_kitchen_table
    p9 = Pose()
    p9.position = Point(x=6.347103, y=1.001381, z=0.817812)
    # From euler angles (rpy) to quaternion
    p9.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    poses['on_kitchen_table'] = {'pose': p9}

    # On bedroom-chair
    p10 = Pose()
    p10.position = Point(x=-8.247623, y=-4.505759, z=0.360569)
    # From euler angles (rpy) to quaternion
    p10.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    poses['on_bedroom_chair'] = {'pose': p10}

    # On orange table
    p11 = Pose()
    p11.position = Point(x=-0.595, y=4.076, z=0.4)
    # From euler angles (rpy) to quaternion
    p11.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    poses['on_orange_table'] = {'pose': p11}

    # On livingroom tabel
    p12 = Pose()
    p12.position = Point(x=1.866698, y=-1.837483, z=0.386063)
    # From euler angles (rpy) to quaternion
    p12.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    poses['on_mainroom_table'] = {'pose': p12}

    # On tv tabel
    p13 = Pose()
    p13.position = Point(x=0.885594, y=-5.203157, z=0.524760)
    # From euler angles (rpy) to quaternion
    q13 = quaternion_from_euler(0, 0, 3.14) 
    p13.orientation = Quaternion(x=q13[0], y=q13[1], z=q13[2], w=q13[3])
    poses['on_tv_table'] = {'pose': p13}

    # On sink
    p14 = Pose()
    p14.position = Point(x=7.078957, y=-5.157995, z=0.913168)
    # From euler angles (rpy) to quaternion
    p14.orientation = Quaternion(x=q13[0], y=q13[1], z=q13[2], w=q13[3])
    poses['on_sink'] = {'pose': p14}


          
            ###______define objects______###
    
    objects = {}

     # add object sphere_r
    f = open(package_path + 'sphere_v/model.sdf', 'r')
    objects['sphere_v'] = {'name': 'sphere_v', 'sdf': f.read()}

    # add object sphere_r
    f = open(package_path + 'sphere_r/model.sdf', 'r')
    objects['sphere_r'] = {'name': 'sphere_r', 'sdf': f.read()}

    # add object person_standing
    f = open(package_path + 'person_standing/model.sdf', 'r')
    objects['person_standing'] = {'name': 'person_standing', 'sdf': f.read()}

    # add object cube_blue
    f = open(package_path + 'cube_b/model.sdf', 'r')
    objects['cube_b'] = {'name': 'cube_b', 'sdf': f.read()}

    # add object cube_blue
    f = open(package_path + 'laptop_pc_1/model.sdf', 'r')
    objects['laptop'] = {'name': 'laptop', 'sdf': f.read()}

    # Check if given object and location are valid

    control = 0
    if not args['location'] in poses.keys():
        print('Location ' + args['location'] +
              ' is unknown. Available locations are ' + str(list(poses.keys())))
        control =1

    if not args['object'] in objects.keys():
        print('Object ' + args['object'] +
              ' is unknown. Available objects are ' + str(list(objects.keys())))
        control =1

    if args['menu']:
        print('Objects :' + str(objects.keys()) +'\n' + 'Locations :' + str(poses.keys()) + '\n')
        control =1
    # -------------------------------
    # ROS
    # -------------------------------

    rospy.init_node('insert_object', log_level=rospy.INFO)
    

    if control == 0:

        print('looking for confirmation...')
        service_name = 'gazebo/spawn_sdf_model'
        print('waiting for service ' + service_name + ' ... ', end='')
        rospy.wait_for_service(service_name)
        print('Found')

        service_client = rospy.ServiceProxy(service_name, SpawnModel)

        print('Spawning an object ...')
        uuid_str = str(uuid.uuid4())
        service_client(objects[args['object']]['name'] + '_' + uuid_str,
                       objects[args['object']]['sdf'],
                       objects[args['object']]['name'] + '_' + uuid_str,
                       poses[args['location']]['pose'],
                       'world')

        print('Done')


if __name__ == '__main__':
    main()