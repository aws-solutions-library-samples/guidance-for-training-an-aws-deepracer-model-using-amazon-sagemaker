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
import math

from matplotlib.collections import PatchCollection
from matplotlib.patches import Rectangle

from shapely.geometry import Point, Polygon
from shapely.geometry.polygon import LinearRing, LineString
from datetime import datetime


EPISODE_PER_ITER = 20

def load_data(fname):
    data = []
    with open(fname, 'r') as f:
        for line in f.readlines():
            if "SIM_TRACE_LOG" in line:
                parts = line.split("SIM_TRACE_LOG:")[1].split('\t')[0].split(",")
                data.append(",".join(parts))
    return data

def convert_to_pandas(data, wpts=None):

    """
    stdout_ = 'SIM_TRACE_LOG:%d,%d,%.4f,%.4f,%.4f,%.2f,%.2f,%d,%.4f,%s,%s,%.4f,%d,%.2f,%s\n' % (
            self.episodes, self.steps, model_location[0], model_location[1], model_heading,
            self.steering_angle,
            self.speed,
            self.action_taken,
            self.reward,
            self.done,
            all_wheels_on_track,
            current_progress,
            closest_waypoint_index,
            self.track_length,
            time.time())
        print(stdout_)
    """        

    df_list = list()
    
    #ignore the first two dummy values that coach throws at the start.
    for d in data[2:]:
        parts = d.rstrip().split(",")
        episode = int(parts[0])
        steps = int(parts[1])
        x = 100*float(parts[2])
        y = 100*float(parts[3])
        ##cWp = get_closest_waypoint(x, y, wpts)
        yaw = float(parts[4])
        steer = float(parts[5])
        throttle = float(parts[6])
        action = float(parts[7])
        reward = float(parts[8])
        done = 0 if 'False' in parts[9] else 1
        all_wheels_on_track = parts[10]
        progress = float(parts[11])
        closest_waypoint = int(parts[12])
        track_len = float(parts[13])
        tstamp = parts[14]
        
        #desired_action = int(parts[10])
        #on_track = 0 if 'False' in parts[12] else 1
        
        iteration = int(episode / EPISODE_PER_ITER) +1
        df_list.append((iteration, episode, steps, x, y, yaw, steer, throttle, action, reward, done, all_wheels_on_track, progress,
                        closest_waypoint, track_len, tstamp))

    header = ['iteration', 'episode', 'steps', 'x', 'y', 'yaw', 'steer', 'throttle', 'action', 'reward', 'done', 'on_track', 'progress', 'closest_waypoint', 'track_len', 'timestamp']
    
    df = pd.DataFrame(df_list, columns=header)
    return df

def episode_parser(data, action_map=True, episode_map=True):
    '''
    Arrange data per episode
    '''
    action_map = {} # Action => [x,y,reward] 
    episode_map = {} # Episode number => [x,y,action,reward] 

 
    for d in data[:]:
        parts = d.rstrip().split("SIM_TRACE_LOG:")[-1].split(",")
        e = int(parts[0])
        x = float(parts[2]) 
        y = float(parts[3])
        angle = float(parts[5])
        ttl = float(parts[6])
        action = int(parts[7])
        reward = float(parts[8])

        try:
            episode_map[e]
        except KeyError:
            episode_map[e] = np.array([0,0,0,0,0,0]) #dummy
        episode_map[e] = np.vstack((episode_map[e], np.array([x,y,action,reward,angle,ttl])))

        try:
            action_map[action]
        except KeyError:
            action_map[action] = []
        action_map[action].append([x, y, reward])
                
    # top laps
    total_rewards = {}
    for x in episode_map.keys():
        arr = episode_map[x]
        total_rewards[x] = np.sum(arr[:,3])

    import operator
    top_idx = dict(sorted(total_rewards.items(), key=operator.itemgetter(1), reverse=True)[:])
    sorted_idx = list(top_idx.keys())

    return action_map, episode_map, sorted_idx

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
    ax.plot(x, y, color='cyan', alpha=0.7, linewidth=3, solid_capstyle='round', zorder=2)

def print_border(ax, waypoints, inner_border_waypoints, outer_border_waypoints):
    line = LineString(waypoints)
    plot_coords(ax, line)
    plot_line(ax, line)

    line = LineString(inner_border_waypoints)
    plot_coords(ax, line)
    plot_line(ax, line)

    line = LineString(outer_border_waypoints)
    plot_coords(ax, line)
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
    
def plot_grid_world(episode_df, inner, outer, scale=10.0, plot=True):
    """
    plot a scaled version of lap, along with throttle taken a each position
    """
    stats = []
    outer = [(val[0] / scale, val[1] / scale) for val in outer]
    inner = [(val[0] / scale, val[1] / scale) for val in inner]

    max_x = int(np.max([val[0] for val in outer]))
    max_y = int(np.max([val[1] for val in outer]))

    print(max_x, max_y)
    grid = np.zeros((max_x+1, max_y+1))

    # create shapely ring for outter and inner
    outer_polygon = Polygon(outer)
    inner_polygon = Polygon(inner)

    print('Outer polygon length = %.2f (meters)' % (outer_polygon.length / scale))
    print('Inner polygon length = %.2f (meters)' % (inner_polygon.length / scale))

    dist = 0.0
    for ii in range(1, len(episode_df)):
        dist += math.sqrt((episode_df['x'].iloc[ii] - episode_df['x'].iloc[ii-1])**2 + (episode_df['y'].iloc[ii] - episode_df['y'].iloc[ii-1])**2)
    dist /= 100.0

   
    t0 = datetime.fromtimestamp(float(episode_df['timestamp'].iloc[0]))
    t1 = datetime.fromtimestamp(float(episode_df['timestamp'].iloc[len(episode_df) - 1]))

    lap_time = (t1-t0).total_seconds()

    average_throttle = np.nanmean(episode_df['throttle'])
    max_throttle = np.nanmax(episode_df['throttle'])
    min_throttle = np.nanmin(episode_df['throttle'])
    velocity = dist/lap_time

    print('Distance, lap time = %.2f (meters), %.2f (sec)' % (dist, lap_time))
    print('Average throttle, velocity = %.2f (Gazebo), %.2f (meters/sec)' % (average_throttle, velocity))

    stats.append((dist, lap_time, velocity, average_throttle, min_throttle, max_throttle))


    if plot == True:
        for y in range(max_y):
            for x in range(max_x):
                point = Point((x, y))

                # this is the track
                if (not inner_polygon.contains(point)) and (outer_polygon.contains(point)):
                    grid[x][y] = -1.0

                # find df slice that fits into this
                df_slice = episode_df[(episode_df['x'] >= (x - 1) * scale) & (episode_df['x'] < x * scale) & \
                                   (episode_df['y'] >= (y - 1) * scale) & (episode_df['y'] < y * scale)]

                if len(df_slice) > 0:
                    #average_throttle = np.nanmean(df_slice['throttle'])
                    grid[x][y] = np.nanmean(df_slice['throttle'])

        fig = plt.figure(figsize=(7,7))
        imgplot = plt.imshow(grid)
        plt.colorbar(orientation='vertical')
        plt.title('Lap time (sec) = %.2f' %lap_time)
        #plt.savefig('grid.png')

    return lap_time, average_throttle, stats
