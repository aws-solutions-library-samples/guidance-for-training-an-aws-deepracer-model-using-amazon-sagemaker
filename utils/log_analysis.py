'''
Copyright 2018 Amazon.com, Inc. or its affiliates. All Rights Reserved.

Permission is hereby granted, free of charge, to any person obtaining a copy of this
software and associated documentation files (the "Software"), to deal in the Software
without restriction, including without limitation the rights to use, copy, modify,
merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
'''



import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import LineString
import pandas as pd
import gzip
import glob

from matplotlib.collections import PatchCollection
from matplotlib.patches import Rectangle

import math

EPISODE_PER_ITER = 20

def load_data(fname):
    data = []
    with open(fname, 'r') as f:
        for line in f.readlines():
            if "SIM_TRACE_LOG" in line:
                parts = line.split("SIM_TRACE_LOG:")[1].split('\t')[0].split(",")
                data.append(",".join(parts))

    return data


def convert_to_pandas(data):
    df_list = list()

    for d in data[:]:
        parts = d.rstrip().split(",")
        episode = int(parts[0])
        steps = int(parts[1])
        x = 100*float(parts[2])
        y = 100*float(parts[3])
        cWp = get_closest_waypoint(x, y, waypoints)
        yaw = float(parts[4])
        steer = float(parts[5])
        throttle = float(parts[6])
        action = float(parts[7])
        reward = float(parts[8])
        progress = float(parts[9])
        desired_action = int(parts[10])
        done = 0 if 'False' in parts[11] else 1
        on_track = 0 if 'False' in parts[12] else 1
        
        iteration = episode % EPISODE_PER_ITER
        df_list.append((iteration, episode, steps, x, y, yaw, steer, throttle, action, reward, progress,
                           desired_action, done, on_track, cWp))

    header = ['iteration', 'episode', 'steps', 'x', 'y', 'yaw', 'steer', 'throttle', 'action', 'reward', 'progress',
                           'desired_action', 'done', 'on_track', 'closeWp']
    return pd.DataFrame(df_list, columns=header)



def make_error_boxes(ax, xdata, ydata, xerror, yerror, facecolor='r',
                     edgecolor='r', alpha=0.3):

    # Create list for all the error patches
    errorboxes = []

    # Loop over data points; create box from errors at each point
    for x, y, xe, ye in zip(xdata, ydata, xerror.T, yerror.T):
        rect = Rectangle((x - xe[0], y - ye[0]), xe.sum(), ye.sum())
        errorboxes.append(rect)

    # Create patch collection with specified colour/alpha
    pc = PatchCollection(errorboxes, facecolor=facecolor, alpha=alpha,
                         edgecolor=edgecolor)

    # Add collection to axes
    ax.add_collection(pc)

    # Plot errorbars
    #artists = ax.errorbar(xdata, ydata, xerr=xerror, yerr=yerror,
    #                      fmt='None', ecolor='k')

    return 0


# Waypoints for re:Invent 2018 Track
waypoints = [(290.80313, 66.51131), (331.82272, 66.51131), (342.07758, 66.51131), (362.5874, 69.07506),
             (418.98932, 66.51131), (449.77963, 69.07506), (454.9071, 66.51131), (531.8188, 69.07506),
             (542.07367, 69.07506), (577.96576, 69.07506), (629.1633, 69.408325), (645.7507, 71.94639),
             (650.6474, 73.2283), (669.5677, 80.61183), (682.5657, 88.63626), (699.69147, 100.27557),
             (710.15137, 118.144714), (715.0736, 126.887), (725.50806, 176.0336), (725.226, 181.16106),
             (724.63635, 186.05774), (708.7926, 228.7694), (699.5888, 240.94707), (671.952, 262.89252),
             (653.3137, 270.91702), (607.70496, 274.8138), (592.297, 275.40353), (571.78723, 275.40353),
             (566.65985, 277.3776), (520.35895, 277.24936), (504.97653, 274.68567), (499.87476, 274.68567),
             (494.36273, 274.68567), (455.03525, 287.63245), (424.50128, 315.8334), (408.45242, 335.52277),
             (400.8125, 348.9824), (376.27765, 374.92725), (368.76596, 388.38678), (353.79382, 402.51288),
             (328.15662, 434.20053), (319.51685, 439.9176), (309.18506, 442.225), (295.05893, 446.78842),
             (281.2148, 451.3519), (281.03534, 448.78815), (250.45015, 450.7622), (224.88977, 447.40375),
             (199.25256, 449.22403), (173.61537, 445.9167), (119.13622, 438.02048), (108.90697, 435.58496),
             (72.527725, 382.38766), (70.7331, 352.59717), (85.11562, 271.53226), (88.65355, 267.14832),
             (89.42268, 251.38138), (94.16555, 241.89566), (102.8053, 201.64522), (102.5233, 191.03139),
             (110.368286, 166.52217), (122.13577, 116.580864), (121.82815, 110.838135), (129.10909, 103.12128),
             (132.03175, 98.99372), (138.64615, 91.353775), (146.20914, 84.611206), (150.3111, 81.662964),
             (203.63654, 69.07506), (275.4208, 69.07506)]

inner_border_waypoints = [(426.29974, 272.02585), (474.30112, 241.0361), (504.592, 237.81934), (622.31824, 238.8192),
                          (639.7226, 236.84634), (656.0023, 230.38272), (670.0201, 219.87991), (680.7966, 206.07144),
                          (687.5791, 189.92201), (689.89386, 172.55975), (687.5791, 155.19746), (680.7965, 139.04803),
                          (676.76697, 120.752556), (656.0023, 114.736725), (639.7226, 108.27313), (622.3182, 106.30031),
                          (192.04852, 106.018295), (179.29834, 108.20985), (168.0065, 114.523674),
                          (159.46298, 124.238525), (154.64378, 136.2446), (107.1815, 359.98975),
                          (110.490074, 379.66956), (120.10409, 394.38873), (134.5449, 404.41602), (151.6955, 408.28125),
                          (275.31827, 411.87048), (286.74628, 410.74765), (296.3474, 407.4328), (304.98483, 402.08835),
                          (312.2359, 394.97552), (400.63306, 286.53006)]
outer_border_waypoints = [(489.51816, 309.09326), (504.08707, 307.53412), (622.403, 308.5596), (657.73224, 304.36154),
                          (690.74084, 291.08682), (719.1398, 269.6559), (740.95984, 241.5549), (754.688, 208.73235),
                          (759.3724, 173.46423), (754.688, 138.1961), (740.95966, 105.37356), (719.1397, 77.27255),
                          (690.7407, 55.84163), (657.73224, 42.56692), (622.40295, 38.368855), (192.08206, 38.18935),
                          (156.3803, 44.305904), (124.76003, 61.97467), (100.83894, 89.1741), (87.35394, 122.79225),
                          (40.899273, 339.32434), (38.33692, 364.29123), (41.268684, 389.21747), (49.553997, 412.9086),
                          (62.795902, 434.2291), (80.35973, 452.1574), (101.40387, 465.83432), (124.919785, 474.6044),
                          (149.78062, 478.04736), (275.05695, 481.66223), (301.9777, 478.85706), (325.98563, 470.57315),
                          (347.58612, 457.21567), (365.723, 439.43765), (454.0945, 331.01785)]



# Reinvent Waypoints

def v_color(ob):
    
    COLOR = {
        True: '#6699cc',
        False: '#ffcc33'
    }

    return COLOR[ob.is_simple]


def plot_coords(ax, ob):
    x, y = ob.xy
    ax.plot(x, y, '.', color='#999999', zorder=1)


def plot_bounds(ax, ob):
    x, y = zip(*list((p.x, p.y) for p in ob.boundary))
    ax.plot(x, y, '.', color='#000000', zorder=1)


def plot_line(ax, ob):
    x, y = ob.xy
    ax.plot(x, y, color=v_color(ob), alpha=0.7, linewidth=3, solid_capstyle='round', zorder=2)


def print_border(ax):
    line = LineString(waypoints)
    plot_coords(ax, line)
    plot_bounds(ax, line)
    plot_line(ax, line)

    line = LineString(inner_border_waypoints)
    plot_coords(ax, line)
    plot_bounds(ax, line)
    plot_line(ax, line)

    line = LineString(outer_border_waypoints)
    plot_coords(ax, line)
    plot_bounds(ax, line)
    plot_line(ax, line)

def get_closest_waypoint(x, y, waypoints):
    res = 0
    index = 0
    min_distance = float('inf')
    for row in waypoints:
        distance = math.sqrt((row[0] - x) * (row[0] - x) + (row[1] - y) * (row[1] - y))
        if distance < min_distance:
            min_distance = distance
            res = index
        index = index + 1
    return res
    
