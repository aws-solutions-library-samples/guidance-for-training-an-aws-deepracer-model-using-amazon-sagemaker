"""
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
"""

import math
from datetime import datetime

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.collections import PatchCollection
from matplotlib.patches import Rectangle
from shapely.geometry import Point, Polygon
from shapely.geometry.polygon import LineString
from sklearn.preprocessing import MinMaxScaler

import cw_utils as cw

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

    # ignore the first two dummy values that coach throws at the start.
    for d in data[2:]:
        parts = d.rstrip().split(",")
        episode = int(parts[0])
        steps = int(parts[1])
        x = 100 * float(parts[2])
        y = 100 * float(parts[3])
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

        iteration = int(episode / EPISODE_PER_ITER) + 1
        df_list.append((iteration, episode, steps, x, y, yaw, steer, throttle,
                        action, reward, done, all_wheels_on_track, progress,
                        closest_waypoint, track_len, tstamp))

    header = ['iteration', 'episode', 'steps', 'x', 'y', 'yaw', 'steer',
              'throttle', 'action', 'reward', 'done', 'on_track', 'progress',
              'closest_waypoint', 'track_len', 'timestamp']

    df = pd.DataFrame(df_list, columns=header)
    return df


def normalize_rewards(df):
    # Normalize the rewards to a 0-1 scale

    min_max_scaler = MinMaxScaler()
    scaled_vals = min_max_scaler.fit_transform(
        df['reward'].values.reshape(df['reward'].values.shape[0], 1))
    df['reward'] = pd.DataFrame(scaled_vals.squeeze())


def episode_parser(data):
    """
    Arrange data per episode
    """
    action_map = {}   # Action => [x,y,reward]
    episode_map = {}  # Episode number => [x,y,action,reward]

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
            episode_map[e] = np.array([0, 0, 0, 0, 0, 0])  # dummy
        episode_map[e] = np.vstack(
            (episode_map[e], np.array([x, y, action, reward, angle, ttl])))

        try:
            action_map[action]
        except KeyError:
            action_map[action] = []
        action_map[action].append([x, y, reward])

    # top laps
    total_rewards = {}
    for x in episode_map.keys():
        arr = episode_map[x]
        total_rewards[x] = np.sum(arr[:, 3])

    import operator
    top_idx = dict(sorted(total_rewards.items(),
                          key=operator.itemgetter(1),
                          reverse=True)[:])
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

    return 0


def v_color(ob):
    color = {
        True: '#6699cc',
        False: '#ffcc33'
    }

    return color[ob.is_simple]


def plot_coords(ax, ob):
    x, y = ob.xy
    ax.plot(x, y, '.', color='#999999', zorder=1)


def plot_bounds(ax, ob):
    x, y = zip(*list((p.x, p.y) for p in ob.boundary))
    ax.plot(x, y, '.', color='#000000', zorder=1)


def plot_line(ax, ob, color='cyan'):
    x, y = ob.xy
    ax.plot(x, y, color=color, alpha=0.7, linewidth=3, solid_capstyle='round',
            zorder=2)


def print_border(ax, waypoints, inner_border_waypoints, outer_border_waypoints,
                 color='lightgrey'):
    line = LineString(waypoints)
    plot_coords(ax, line)
    plot_line(ax, line, color)

    line = LineString(inner_border_waypoints)
    plot_coords(ax, line)
    plot_line(ax, line, color)

    line = LineString(outer_border_waypoints)
    plot_coords(ax, line)
    plot_line(ax, line, color)


def plot_top_laps(sorted_idx, episode_map, center_line, inner_border,
                  outer_border, n_laps=5):
    fig = plt.figure(n_laps, figsize=(12, 30))
    for i in range(n_laps):
        idx = sorted_idx[i]

        episode_data = episode_map[idx]

        ax = fig.add_subplot(n_laps, 1, i + 1)

        line = LineString(center_line)
        plot_coords(ax, line)
        plot_line(ax, line)

        line = LineString(inner_border)
        plot_coords(ax, line)
        plot_line(ax, line)

        line = LineString(outer_border)
        plot_coords(ax, line)
        plot_line(ax, line)

        for idx in range(1, len(episode_data) - 1):
            x1, y1, action, reward, angle, speed = episode_data[idx]
            car_x2, car_y2 = x1 - 0.02, y1
            plt.plot([x1 * 100, car_x2 * 100], [y1 * 100, car_y2 * 100], 'b.')

    return fig


def plot_grid_world(episode_df, inner, outer, scale=10.0, plot=True,
                    log_tuple=None, min_distance_to_plot=None,
                    graphed_value='throttle'):
    """
    plot a scaled version of lap, along with throttle taken a each position
    """
    stats = []
    outer = [(val[0] / scale, val[1] / scale) for val in outer]
    inner = [(val[0] / scale, val[1] / scale) for val in inner]

    max_x = int(np.max([val[0] for val in outer]))
    max_y = int(np.max([val[1] for val in outer]))
    min_x = min(int(np.min([val[0] for val in outer])), 0)
    min_y = min(int(np.min([val[1] for val in outer])), 0)

    outer = [(val[0] - min_x, val[1] - min_y) for val in outer]
    inner = [(val[0] - min_x, val[1] - min_y) for val in inner]

    grid = np.zeros((max_x + 1 - min_x, max_y + 1 - min_y))

    # create shapely ring for outter and inner
    outer_polygon = Polygon(outer)
    inner_polygon = Polygon(inner)

    print('Outer polygon length = %.2f (meters)' % (
            outer_polygon.length / scale))
    print('Inner polygon length = %.2f (meters)' % (
            inner_polygon.length / scale))

    dist = 0.0
    for ii in range(1, len(episode_df)):
        dist += math.sqrt(
            (episode_df['x'].iloc[ii] - episode_df['x'].iloc[ii - 1]) ** 2 + (
                    episode_df['y'].iloc[ii] - episode_df['y'].iloc[
                ii - 1]) ** 2)
    dist /= 100.0

    t0 = datetime.fromtimestamp(float(episode_df['timestamp'].iloc[0]))
    t1 = datetime.fromtimestamp(
        float(episode_df['timestamp'].iloc[len(episode_df) - 1]))

    lap_time = (t1 - t0).total_seconds()

    average_throttle = np.nanmean(episode_df['throttle'])
    max_throttle = np.nanmax(episode_df['throttle'])
    min_throttle = np.nanmin(episode_df['throttle'])
    velocity = dist / lap_time

    distance_lap_time = 'Distance, lap time = %.2f (meters), %.2f (sec)' % (
        dist, lap_time)
    print(distance_lap_time)
    throttle_velocity = 'Average throttle, velocity = %.2f (Gazebo), %.2f (meters/sec)' % (
        average_throttle, velocity)
    print(throttle_velocity)

    stats.append((dist, lap_time, velocity, average_throttle, min_throttle,
                  max_throttle))

    if plot == True and (not min_distance_to_plot or lap_time > min_distance_to_plot):
        for y in range(max_y - min_y):
            for x in range(max_x - min_x):
                point = Point((x, y))

                # this is the track
                if (not inner_polygon.contains(point)) and (
                        outer_polygon.contains(point)):
                    grid[x][y] = -1.0

                # find df slice that fits into this
                df_slice = episode_df[
                    (episode_df['x'] >= (x + min_x - 1) * scale) & (
                            episode_df['x'] < (x + min_x) * scale) & \
                    (episode_df['y'] >= (y + min_y - 1) * scale) & (
                            episode_df['y'] < (y + min_y) * scale)]

                if len(df_slice) > 0:
                    # average_throttle = np.nanmean(df_slice['throttle'])
                    grid[x][y] = np.nanmean(df_slice[graphed_value])

        fig = plt.figure(figsize=(12, 16))
        imgplot = plt.imshow(grid)
        subtitle = ''
        if log_tuple:
            subtitle = '\n%s\n%s\n%s\n%s' % (
                log_tuple[1], datetime.fromtimestamp(log_tuple[2] / 1000.0),
                distance_lap_time, throttle_velocity)
        plt.colorbar(orientation='horizontal')
        plt.title('Lap time (sec) = %.3f%s' % (lap_time, subtitle))
        plt.show()
        plt.clf()

    return lap_time, average_throttle, stats


def simulation_agg(panda, firstgroup='iteration', add_timestamp=False):
    grouped = panda.groupby([firstgroup, 'episode'])

    by_steps = grouped['steps'].agg(np.max).reset_index()
    if 'new_reward' not in panda.columns:
        print('new reward not found, using reward as its values')
        panda['new_reward'] = panda['reward']
    by_new_reward = grouped['new_reward'].agg(np.sum).reset_index()
    by_reward = grouped['reward'].agg(np.sum).reset_index()
    by_progress = grouped['progress'].agg(np.max).reset_index()
    by_throttle = grouped['throttle'].agg(np.mean).reset_index()
    by_time = grouped['timestamp'].agg(np.ptp).reset_index() \
        .rename(index=str, columns={"timestamp": "time"})
    by_time['time'] = by_time['time'].astype(float)

    result = by_steps \
        .merge(by_progress, on=[firstgroup, 'episode']) \
        .merge(by_time, on=[firstgroup, 'episode']) \
        .merge(by_new_reward, on=[firstgroup, 'episode']) \
        .merge(by_throttle, on=[firstgroup, 'episode']) \
        .merge(by_reward, on=[firstgroup, 'episode'])

    result['time_if_complete'] = result['time'] * 100 / result['progress']
    result['reward_if_complete'] = result['reward'] * 100 / result['progress']
    result['quintile'] = pd.cut(result['episode'], 5, labels=['1st', '2nd', '3rd', '4th', '5th'])

    if add_timestamp:
        by_timestamp = grouped['timestamp'].agg(np.max).astype(float).reset_index()
        result = result.merge(by_timestamp, on=[firstgroup, 'episode'])

    return result


def scatter_aggregates(aggregate_df, title=None):
    fig, axes = plt.subplots(nrows=2, ncols=3, figsize=[15, 7.2])
    if title:
        fig.suptitle(title)
    aggregate_df.plot.scatter('time', 'reward', ax=axes[0, 0])
    aggregate_df.plot.scatter('time', 'new_reward', ax=axes[1, 0])
    aggregate_df.plot.scatter('time', 'progress', ax=axes[0, 1])
    aggregate_df.plot.scatter('time', 'steps', ax=axes[0, 2])
    aggregate_df.hist(column=['time'], bins=20, ax=axes[1, 1])
    aggregate_df.hist(column=['progress'], bins=20, ax=axes[1, 2])


def analyze_categories(panda, category='quintile', groupcount=5):
    grouped = panda.groupby(category)

    fig, axes = plt.subplots(nrows=groupcount, ncols=4, figsize=[15, 15])

    row = 0
    for name, group in grouped:
        group.plot.scatter('time', 'reward', ax=axes[row, 0])
        group.plot.scatter('time', 'new_reward', ax=axes[row, 1])
        group.hist(column=['time'], bins=20, ax=axes[row, 2])
        group.hist(column=['progress'], bins=20, ax=axes[row, 3])
        row += 1


def avg_and_dev(values, episodes_per_iteration):
    average_val_per_iteration = list()
    deviation_val_per_iteration = list()

    buffer_val = list()
    for val in values:
        buffer_val.append(val)

        if len(buffer_val) == episodes_per_iteration:
            average_val_per_iteration.append(np.mean(buffer_val))
            deviation_val_per_iteration.append(np.std(buffer_val))
            # reset
            buffer_val = list()

    return average_val_per_iteration, deviation_val_per_iteration


def plot(ax, values, xlabel, ylabel, title=None, red_above=None):
    ax.plot(np.arange(len(values)), values, '.')
    if title:
        ax.set_title(title)
    ax.set_ylabel(ylabel)
    ax.set_xlabel(xlabel)

    if red_above:
        for rr in range(len(values)):
            if values[rr] >= red_above:
                ax.plot(rr, values[rr], 'r.')

    plt.grid(True)


def completion_rate(progresses):
    completes = [progress for progress in progresses if progress == 100.0]
    return len(completes) / len(progresses)


def analyze_training_progress(panda, episodes_per_iteration):
    # reward graph per episode
    min_episodes = np.min(panda['episode'])
    max_episodes = np.max(panda['episode'])
    print('Number of episodes = ', max_episodes)

    total_reward_per_episode = list()
    time_per_episode = list()
    completed_time_per_episode = list()
    progress_per_episode = list()
    for epi in range(min_episodes, max_episodes):
        df_slice = panda[panda['episode'] == epi]
        total_reward_per_episode.append(np.sum(df_slice['reward']))
        time_per_episode.append(np.ptp(df_slice['timestamp']))
        progress_per_episode.append(np.max(df_slice['progress']))
        completed_time_per_episode.append(time_per_episode[-1] if progress_per_episode[-1] == 100.0 else 0)

    average_reward_per_iteration, deviation_reward_per_iteration = avg_and_dev(total_reward_per_episode,
                                                                               episodes_per_iteration)
    average_time_per_iteration, deviation_time_per_iteration = avg_and_dev(time_per_episode, episodes_per_iteration)
    average_progress_per_iteration, deviation_progress_per_iteration = avg_and_dev(progress_per_episode,
                                                                                   episodes_per_iteration)

    completion_rate_per_iteration = list()

    total_completion_rate = completion_rate(progress_per_episode)

    buffer_val = list()
    iter_count = 0
    for val in progress_per_episode:
        buffer_val.append(val)

        if len(buffer_val) == episodes_per_iteration:
            completion_rate_for_iteration = completion_rate(buffer_val)
            completion_rate_per_iteration.append(completion_rate_for_iteration)
            buffer_val = list()
            iter_count += 1

    completed_time_per_iteration = list()
    buffer_val = list()
    for val in completed_time_per_episode:
        buffer_val.append(val)

        if len(buffer_val) == episodes_per_iteration:
            complete_times = [t for t in buffer_val if t != 0]
            buffer_val = list()
            if len(complete_times) > 0:
                completed_time_per_iteration.append(np.mean(complete_times))
            else:
                completed_time_per_iteration.append(0)

    print('Number of iterations = ', iter_count)

    fig, axes = plt.subplots(nrows=3, ncols=3, figsize=[15, 15])

    ax = axes[0, 0]
    plot(ax, average_reward_per_iteration, 'Iteration', 'Mean reward', 'Rewards per Iteration')

    ax = axes[1, 0]
    plot(ax, deviation_reward_per_iteration, 'Iteration', 'Dev of reward')

    ax = axes[2, 0]
    plot(ax, total_reward_per_episode, 'Episode', 'Total reward')

    ax = axes[0, 1]
    plot(ax, average_time_per_iteration, 'Iteration', 'Mean time', 'Times per Iteration')

    ax = axes[1, 1]
    plot(ax, deviation_time_per_iteration, 'Iteration', 'Dev of time')

    ax = axes[2, 1]
    plot(ax, completed_time_per_iteration, 'Iteration', 'Mean completed laps time', 'Mean completed time')

    ax = axes[0, 2]
    plot(ax, average_progress_per_iteration, 'Iteration', 'Mean progress', 'Progress per Iteration')

    ax = axes[1, 2]
    plot(ax, deviation_progress_per_iteration, 'Iteration', 'Dev of progress')

    ax = axes[2, 2]
    plot(ax, completion_rate_per_iteration, 'Iteration', 'Completion rate',
         'Completion rate (avg: %s)' % total_completion_rate)


def load_eval_data(eval_fname):
    eval_data = load_data(eval_fname)
    return convert_to_pandas(eval_data)


def load_eval_logs(logs):
    full_dataframe = None
    for log in logs:
        eval_data = load_data(log[0])
        dataframe = convert_to_pandas(eval_data)
        dataframe['stream'] = log[1]
        if full_dataframe is not None:
            full_dataframe = full_dataframe.append(dataframe)
        else:
            full_dataframe = dataframe

    return full_dataframe.sort_values(
        ['stream', 'episode', 'steps']).reset_index()


def analyse_single_evaluation(log_file, inner_border, outer_border, episodes=5,
                              log_tuple=None, min_distance_to_plot=None):
    print("###############################################################")
    print(log_file)
    eval_df = load_eval_data(log_file)

    for e in range(episodes):
        print("\nEpisode #%s " % e)
        episode_df = eval_df[eval_df['episode'] == e]
        plot_grid_world(episode_df, inner_border, outer_border, scale=5.0,
                        log_tuple=log_tuple, min_distance_to_plot=min_distance_to_plot)


def analyse_multiple_race_evaluations(logs, inner_border, outer_border, min_distance_to_plot=None):
    for log in logs:
        analyse_single_evaluation(log[0], inner_border, outer_border,
                                  log_tuple=log, min_distance_to_plot=min_distance_to_plot)


def download_and_analyse_multiple_race_evaluations(log_folder, l_inner_border, l_outer_border, not_older_than=None,
                                                   older_than=None,
                                                   log_group='/aws/deepracer/leaderboard/SimulationJobs',
                                                   min_distance_to_plot=None):
    logs = cw.download_all_logs("%s/deepracer-eval-" % log_folder, log_group, not_older_than, older_than)

    analyse_multiple_race_evaluations(logs, l_inner_border, l_outer_border, min_distance_to_plot=min_distance_to_plot)
