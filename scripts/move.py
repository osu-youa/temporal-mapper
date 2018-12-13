#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import pi
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Quaternion

WAYPOINTS = [
    (2.5, 0, -pi/2),
    (2.5, 1.5, -pi/2),
    (5, 0, pi),
    (5, -5, pi),
    (2.5, -5, pi/2),
    (0, -5, pi/2),
    (0, 0, 0),
]

def movebase_client():

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server(rospy.Duration(10))

    for x, y, theta in WAYPOINTS:
        rospy.loginfo('Heading to point ({}, {})...'.format(x,y))

        q = quaternion_from_euler(0, 0, theta)

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "/map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        # goal.target_pose.pose.orientation.w = 1.0
        goal.target_pose.pose.orientation = Quaternion(*q)

        import time
        time.sleep(1)

        client.send_goal(goal)
        success = client.wait_for_result(rospy.Duration(60))

        if not success:

            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")

        rospy.loginfo('Reached point ({}, {})!'.format(x, y))

        # else:
        #     return client.get_result()

if __name__ == '__main__':
    rospy.init_node('movebase_client_py')
    try:
        result = movebase_client()
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")