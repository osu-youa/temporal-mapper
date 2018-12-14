#!/usr/bin/env python

import numpy as np
import hdbscan


def run_cluster_analysis(data, min_cluster_size=5):
    # hdbscan returns -1 for points that it considers to be noise and categories 0 through N-1 for the N categories
    return hdbscan.HDBSCAN(min_cluster_size=min_cluster_size).fit_predict(data)

def get_points_com(point_array, masses=None):

    if not isinstance(point_array, np.ndarray):
        point_array = np.array(point_array)

    if masses is None:
        masses = np.ones(np.shape(point_array)[0])
    else:
        if np.shape(point_array)[0] != len(masses):
            raise ValueError('The masses vector had an inconsistent length with the points')

    return np.dot(point_array.T, masses) / np.sum(masses)

def stare_at_point_cluster(points, distance, candidates = 16):
    radial_candidates = np.linspace(0, 2*np.pi, candidates, endpoint=False)
    com = get_points_com(points)

    points_x, points_y = com[0] + np.cos(radial_candidates) * distance, com[1] + np.sin(radial_candidates) * distance


    sum_squares = []
    for x, y in zip(points_x, points_y):
        squared_distances = (points_x - x) ** 2 + (points_y - y) ** 2
        sum_squares.append(squared_distances.sum())

    final_rez_array = np.array([points_x, points_y, sum_squares]).T
    sorted = final_rez_array[np.argsort(final_rez_array[2])]

    return sorted
