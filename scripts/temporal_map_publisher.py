#!/usr/bin/env python

import matplotlib.pyplot as plt
import cPickle
import pdb
import numpy as np
import stats_autocorr
import getpass
import world_publisher as wp
from collections import defaultdict
import rospy
import message_filters
from nav_msgs.msg import OccupancyGrid
from world_publisher import construct_map
from time import time

GLOBAL_TIME_SERIES = defaultdict(dict) # {cell: {t1: 0.3, t2: 3.4, t3: 1.2}}
AUTOCORR_CACHE = {}
REQUIRES_UPDATE = set()
INTERPOLATION_INTERVAL = 0.2

file_name = '/home/{}/temporal_data.pickle'.format(getpass.getuser())

pub = rospy.Publisher('temporal_map', OccupancyGrid, queue_size=1)

CYCLE_DELAY = 5
CYCLE_DELAY_COUNTER = 0

LAST_UPDATE = 0

def output_dict(path='/home/{}/temporal_data.pickle'.format(getpass.getuser()), frequency=5):
    global LAST_UPDATE
    current_time = time()
    if current_time - LAST_UPDATE > 5:
        LAST_UPDATE = current_time
        with open(path, 'wb') as fh:
            current_ser = GLOBAL_TIME_SERIES.copy()
            cPickle.dump(current_ser, fh)


def run_autocorrelation_analysis(occupancy_data):

    cell_data = unpack_map(occupancy_data)
    map_time = occupancy_data.header.stamp.secs + occupancy_data.header.stamp.nsecs / 1.e9

    for point in cell_data:
        GLOBAL_TIME_SERIES[point][map_time] = cell_data[point]
        REQUIRES_UPDATE.add(point)

    global CYCLE_DELAY_COUNTER
    if not CYCLE_DELAY_COUNTER:
        update_autocorrs()
        publish_autocorr_map()

        output_dict()

    CYCLE_DELAY_COUNTER = (CYCLE_DELAY_COUNTER + 1) % (CYCLE_DELAY + 1)

def update_autocorrs():
    for point in REQUIRES_UPDATE:

        ts_data = np.array(list(GLOBAL_TIME_SERIES[point].iteritems()))
        sort_index = np.argsort(ts_data[:, 0])
        ts_data = ts_data[sort_index]

        interpolation, steps, mask = stats_autocorr.linearly_interpolate_data(ts_data[:, 0], ts_data[:, 1], INTERPOLATION_INTERVAL)
        if len(interpolation) < 10:
            continue

        _, autocorrs = stats_autocorr.get_autocorrelations_spectral(interpolation, data_mask=mask)

        if len(autocorrs):
            autocorr = max(autocorrs)
        else:
            autocorr = 0.0

        AUTOCORR_CACHE[point] = autocorr

    global REQUIRES_UPDATE
    REQUIRES_UPDATE = set()



def publish_autocorr_map():

    if not AUTOCORR_CACHE:
        return

    map = construct_map(AUTOCORR_CACHE)
    if map is not None:
        pub.publish(map)

def unpack_map(occupancy_data):

    new_cells = {}

    res = occupancy_data.info.resolution
    x_offset = int(round(occupancy_data.info.origin.position.x / res))
    y_offset = int(round(occupancy_data.info.origin.position.y / res))

    for i, value in enumerate(occupancy_data.data):
        if value < 0:
            continue

        x_cell = x_offset + i % occupancy_data.info.width
        y_cell = y_offset + i // occupancy_data.info.width

        new_cells[x_cell, y_cell] = value / 100.0

    return new_cells

if __name__ == '__main__':
    rospy.init_node('mapper')
    occupancy_sub = message_filters.Subscriber('map_cell', OccupancyGrid, queue_size=1)
    occupancy_sub.registerCallback(run_autocorrelation_analysis)
    rospy.spin()



#
# if __name__ == '__main__':
#     rospy.init_node('temporal_map_publisher')
#     rate = rospy.Rate(500)
#     while not rospy.is_shutdown():
#
#         point_autocorrs = run_autocorrelation_analysis()
#         map_data = wp.construct_map(point_autocorrs)
#         if map_data:
#             pub.publish(map_data)
#         rate.sleep()