#!/usr/bin/env python

# Every python controller needs these lines
import rospy
import os
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Point, Pose, Quaternion
import xml.etree.ElementTree as ET
from sensor_msgs.msg import LaserScan
import numpy as np
import bresenham
from nav_msgs.msg import OccupancyGrid, MapMetaData

from collections import defaultdict, Counter
import message_filters
import pdb
from time import time
import cPickle

RESOLUTION = 0.25
UPDATE_FREQUENCY = 20
MIN_OBS_THRESHOLD = 5

pub = rospy.Publisher('map_cell', OccupancyGrid, queue_size=1)

def callback(odom_msg, scan_msg):
    """
    This function takes in an odometry reading (i.e. the robot's position in the world) and a set of laser scans.
    It then discretizes the world into a grid cell whose resolution is defined in the config and classifies grid cells
    as empty or nonempty based on whether or not laser scans pass through those cells.
    It then updates the global time series with the current time's occupancies.

    :param odom_msg:
    :param scan_msg:
    :return:
    """


    _, _, heading = euler_from_quaternion(
        [odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z,
         odom_msg.pose.pose.orientation.w])

    position = odom_msg.pose.pose.position
    x = position.x
    y = position.y
    readings = np.array(scan_msg.ranges)
    angles = heading + np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(readings))

    # Lasers going past the max range will return NaN, so we take note of these for our occupancy later
    valid_indicator = ~np.isnan(readings)
    readings = readings[valid_indicator]
    angles = angles[valid_indicator]

    endpoints_x, endpoints_y = x+np.cos(angles)*readings, y+np.sin(angles)*readings

    # Cell refers to the discretized coordinate we will store the data in
    start_xy = np.round([x/RESOLUTION, y/RESOLUTION]).astype(np.int)
    cells_x = np.round(endpoints_x/RESOLUTION).astype(np.int)
    cells_y = np.round(endpoints_y/RESOLUTION).astype(np.int)

    cell_xy_pair_counts = Counter(zip(cells_x, cells_y))

    current_occupancies = defaultdict(list)

    # We draw a line between the robot's current position's cell and the cell of the laser endpoint. We then mark
    # all intermediate cells as unoccupied and the final cell as occupied, unless we have a max_range observation,
    # in which case all observations are marked as unoccupied

    for end_xy, xy_count in cell_xy_pair_counts.iteritems():
        matched_cells = list(bresenham.covered_cells(start_xy, end_xy))
        for cell in matched_cells[:-1]:
            current_occupancies[cell].extend([0.] * xy_count)
        current_occupancies[matched_cells[-1]].extend([1.0] * xy_count)

    new_map = construct_map(occupancies=current_occupancies)
    if new_map is not None:
        pub.publish(new_map)

def construct_map(occupancies):

    if not occupancies:
        return

    new_map = OccupancyGrid()

    # Construct some of the metadata necessary to send/format the map
    all_cell_pairs = occupancies.keys()
    all_x = [p[0] for p in all_cell_pairs]
    all_y = [p[1] for p in all_cell_pairs]
    x_min, x_max, y_min, y_max = min(all_x), max(all_x), min(all_y), max(all_y)
    x_range, y_range = x_max - x_min, y_max - y_min
    convert_cell_to_list_index = lambda p: (p[1] - y_min) * (x_range + 1) + (p[0] - x_min)

    # Construct header
    new_map.header.frame_id = '/map'
    new_map.header.stamp = rospy.Time.now()

    # Construct MapMetaData object
    new_map.info.resolution = RESOLUTION
    new_map.info.width = x_range + 1
    new_map.info.height = y_range + 1
    new_map.info.origin.position.x = x_min * RESOLUTION
    new_map.info.origin.position.y = y_min * RESOLUTION
    new_map.info.origin.orientation.w = 1.0

    # Construct the list
    final_list = [-1] * ((x_range + 1) * (y_range + 1))
    for cell_pair in all_cell_pairs:
        list_index = convert_cell_to_list_index(cell_pair)
        final_list[list_index] = np.floor(100*np.mean(occupancies[cell_pair]))

    new_map.data = final_list

    return new_map


if __name__ == '__main__':
    rospy.init_node('mapper')
    odom_sub = message_filters.Subscriber('/odom', Odometry)
    lidar_sub = message_filters.Subscriber('/scan', LaserScan)
    sub = message_filters.TimeSynchronizer([odom_sub, lidar_sub], UPDATE_FREQUENCY)
    sub.registerCallback(callback)
    rospy.spin()