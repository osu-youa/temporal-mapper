#!/usr/bin/env python

# Every python controller needs these lines
import rospy
import os
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Point, Pose, Quaternion
import xml.etree.ElementTree as ET
import getpass

CYCLE_TIME = 10.0
OBJECTS = {
    'door_stopper': {
        'interval': [1.0, 3.0],
        'pose': (0, 0, 0),
        'sdf': ''
    },
    'door_stopper_0': {
        'interval': [9.0, 1.0],
        'pose': (0, 0, 0),
        'sdf': ''
    },
}
STATUS = {}

WORLD_PATH = '/home/{}/gazebo/whippu_dungeon.world'.format(getpass.getuser())


def get_object_desired_status(obj, time_elapsed):
    start, end = OBJECTS[obj]['interval']

    if start < end:
        return start <= time_elapsed < end
    else:
        return (time_elapsed >= start) or (time_elapsed < end)

def load_sdfs_from_world():
    """
    This function reads in the world file that is being used and extracts all of the models of interest out of it so
    that their SDFs can be fed into spawn_sdf_model.
    """

    with open(WORLD_PATH, 'r') as fh:
        world_xml = ET.fromstring(''.join(fh.readlines()))

    processed = 0

    for element in filter(lambda e: e.tag == 'model', world_xml[0]):
        name = element.get('name')
        if name not in OBJECTS:
            continue

        # Extract the pose out of the element
        # Hardcoding in indexes, might be better to fix later



        pose_value = element.find('pose').text.split(' ')
        OBJECTS[name]['pose'] = (float(pose_value[0]), float(pose_value[1]), float(pose_value[2]))

        sdf_tag = ET.Element('sdf', attrib={'version': '1.6'})
        sdf_tag.insert(0, element)

        OBJECTS[name]['sdf'] = ET.tostring(sdf_tag)

        processed += 1

    if processed != len(OBJECTS):
        raise Exception('Some of the desired objects do not exist in the world file!')

def initialize_world():

    # Once everything has been processed, we wait for Gazebo to initialize to remove objects from the world
    rospy.wait_for_service('gazebo/delete_model')
    for obj in OBJECTS:

        spawn_object = get_object_desired_status(obj, 0)

        if not spawn_object:
            try:
                rospy.ServiceProxy('gazebo/delete_model', DeleteModel)(obj)
            except:
                pass

        STATUS[obj] = spawn_object

if __name__ == '__main__':



    rospy.init_node('object_manager')

    load_sdfs_from_world()
    initialize_world()

    block_1_arg = rospy.get_param("/object_manager/block_1", None)
    block_2_arg = rospy.get_param("/object_manager/block_2", None)

    rospy.loginfo("Values of block_1 and block_2 are {} and {}".format(block_1_arg, block_2_arg))

    # Loop at 10Hz
    rate = rospy.Rate(10)

    all_objs = list(OBJECTS.keys())

    base_time = rospy.Time.now().to_time()

    while not rospy.is_shutdown():
        current_time = rospy.Time.now().to_time()
        cycle_elapsed = (current_time - base_time) % CYCLE_TIME

        for obj in all_objs:
            active = get_object_desired_status(obj, cycle_elapsed)
            previous_active = STATUS[obj]

            if previous_active != active:
                try:
                    if previous_active:
                        # Despawn
                        rospy.loginfo('Despawning {}'.format(obj))
                        rospy.wait_for_service('gazebo/delete_model')
                        srv = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
                        srv(obj)

                    else:
                        rospy.loginfo('Spawning {}'.format(obj))
                        pose = Pose(Point(*OBJECTS[obj]['pose']), Quaternion())
                        rospy.loginfo(pose)
                        rospy.wait_for_service('gazebo/spawn_sdf_model')
                        srv = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
                        srv(obj, OBJECTS[obj]['sdf'], 'DUMMY', pose, '')

                    STATUS[obj] = active

                except Exception as e:
                    rospy.loginfo('Exception raised: {}'.format(e))
