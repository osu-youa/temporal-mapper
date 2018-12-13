#!/usr/bin/env python

import rospy
import pdb
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped



pub = rospy.Publisher('tf', TFMessage, queue_size=1)

def tf_callback(tf_msg):

    """
    Subscribe to the global /tf topic and republish all transforms in the local namespace which do not correspond to
    odom->map transforms. Must be initialized in its own namespace to have any useful effect.

    :param tf_msg:
    :return:
    """


    tfs_to_keep = []
    tfs = tf_msg.transforms

    for tf in tfs:
        if tf.child_frame_id == 'odom' and tf.header.frame_id == 'map':
            rospy.loginfo('Discarded!')
            continue
        tfs_to_keep.append(tf)

    if not tfs_to_keep:
        return

    tf_msg.transforms = tfs_to_keep
    pub.publish(tfs_to_keep)


if __name__ == '__main__':
    rospy.init_node('tf_filter')
    rospy.Subscriber('/tf', TFMessage, tf_callback)
    rospy.spin()
