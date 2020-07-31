---
title: "Modify the sensor combinations"
chapter: true
weight: 15
description: "Explore the possible sensor combinations in the simulation application."
---

# Modify the sensor combinations

Note that we have already added all the sensor combinations into the simulation application, and thus you only have to specify the sensor that you want to use. 


### Possible sensor configuration

Single camera

Single camera + LIDAR

Stereo cameras

Stereo cameras + LIDAR

When you specify a new sensor configuration this impacts the state data that will feed into the neural network.



![Image](/images/400workshop/networkinput.png)

### Camera

Single-lens 120-degree field of view camera capturing at 15fps. The images are converted into greyscale before being fed to the neural network.

### Stereo cameras

Composed of two single-lens cameras, stereo camera can generate depth information of the objects in front of the agent and thus be used to detect and avoid obstacles on the track. The cameras capture images with the same resolution and frequency. Images from both cameras are converted into grey scale, stacked and then fed into the neural network.

![Image](/images/400workshop/inputembedder.png)

### LIDAR sensor

LIDAR is a light detection and ranging sensor. It scans its environment and provides inputs to the model to determine when to overtake another vehicle and beat it to the finish line. It provides continuous visibility of its surroundings and can see in all directions and always know its distances from objects or other vehicles on the track.



### Configure the sensor in the action space artifact file

In the action space artifact file there is an entry for sensor.

![Image](/images/400workshop/actionspaceexamplesensor.png)

For the sensor you have the following choices:

`"sensor": ["FRONT_FACING_CAMERA"]`

`"sensor": ["FRONT_FACING_CAMERA", "LIDAR"]`

`"sensor": ["STEREO_CAMERAS"]`

`"sensor": ["STEREO_CAMERAS", "LIDAR"]`

Modify the action space artifact file you created in the previous exercise by adding the value for sensor.

### Save your action space artifact file and move to the next activity.

**[Proceed to the next activity](../modifyneuralnetwork/)**
