import numpy as np

# Shapely Library
from shapely.geometry import Polygon
from shapely.geometry.polygon import LineString


def get_track_waypoints(track_name, absolute_path="."):
    return np.load("%s/tracks/%s.npy" % (absolute_path, track_name))


def load_track(track_name, absolute_path="."):
    if track_name.endswith('.npy'):
        track_name = track_name[:-4]

    waypoints = get_track_waypoints(track_name, absolute_path)

    print("Loaded %s waypoints" % waypoints.shape[0])

    l_inner_border = LineString(waypoints[:, 2:4])
    l_outer_border = LineString(waypoints[:, 4:6])
    road_poly = Polygon(np.vstack((l_outer_border, np.flipud(l_inner_border))))

    # rescale waypoints to centimeter scale
    center_line = waypoints[:, 0:2] * 100
    inner_border = waypoints[:, 2:4] * 100
    outer_border = waypoints[:, 4:6] * 100

    return center_line, inner_border, outer_border, road_poly


def plot_trackpoints(trackpoints, show=True):
    import matplotlib.pyplot as plt
    for point in trackpoints:
        plt.scatter(point[0], point[1], c="blue")
        plt.scatter(point[2], point[3], c="black")
        plt.scatter(point[4], point[5], c="cyan")
    if show:
        plt.show()
