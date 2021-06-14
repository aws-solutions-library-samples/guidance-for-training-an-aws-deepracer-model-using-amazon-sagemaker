# Track Data

The files in this directory contain the waypoints of various tracks.

You can obtain the waypoints at the centre of the lane by printing them in your reward function.

To load an `.npy` file, use `numpy`:

```
import numpy as np
data = np.load("x.npy")
```

There is one row per waypoint.
There are 6 columns.

1. x coordinate of centre lane at waypoint
2. y coordinate of centre lane at waypoint
3. x coordinate of left lane beside waypoint
4. y coordinate of left lane beside waypoint
5. x coordinate of right lane
6. y coordinate of right lane

The units are in meters.

