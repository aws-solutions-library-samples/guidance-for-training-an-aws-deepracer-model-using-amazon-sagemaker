---
title: "Update the neural network architecture"
chapter: true
weight: 20
description: "We have partnered with Intel and use Coach as reinforcement learning framework. Furthermore, we are using Tensorflow as our deep learning framework.

The neural network architecture typically includes an input embedder, middleware, and an output head, see descriptions here. In this section we are interested in changing the middleware."
---

# Update the neural network architecture

We have partnered with Intel and use Coach as reinforcement learning framework. Furthermore, we are using Tensorflow as our deep learning framework. 

The neural network architecture typically includes an input embedder, middleware, and an output head. In this section we are interested in changing the middleware.

### Network Design
Each agent has at least one neural network, used as the function approximator, for choosing the actions. The network is designed in a modular way to allow reusability in different agents. 

It is separated into three main parts:

![Image](/images/400workshop/coachnames.png)

#### CNN image input embedders 
This is the first stage of the network, meant to convert the input into a feature vector representation. It is possible to combine several instances of any of the supported embedders, in order to allow varied combinations of inputs.

The type of Input Embedder that AWS DeepRacer uses is Convolutional Neural Network.

#### FC middleware
The middleware gets the output of the input embedder, and processes it into a different representation domain, before sending it through the output head. The goal of the middleware is to enable processing the combined outputs of several input embedders, and pass them through some extra processing. This, for instance, might include an LSTM or just a plain simple FC layer.

#### Action output
The output head is used in order to predict the values required from the network. These might include action-values, state-values or a policy. As with the input embedders, it is possible to use several output heads in the same network. For example, the Actor Critic agent combines two heads - a policy head and a state-value head. In addition, the output heads defines the loss function according to the head type.

The default action space for our RL agent is discrete, therefore, the number of actions correspond to the number of output nodes of the policy network.


### Exercise  - Update the Neural Network Architecture

In the action space artifact file there is an entry for neural_network. 

![Image](/images/400workshop/actionspaceexamplenetwork.png)

When updating the neural_network entry take into account the different combinations of sensors and network depth outlined below.

Neural Network values available for use in your action space artifact file:

`["DEEP_CONVOLUTIONAL_NETWORK"]`

`["DEEP_CONVOLUTIONAL_NETWORK_SHALLOW"]`

`["DEEP_CONVOLUTIONAL_NETWORK_DEEP"]`


In the example of a Front Facing Camera using a neural network value of Deep Convolutional Network we have four convolutional layers on top of each other. The first one has 32 convolutional filters, a kernel size of 5x5, and a stride length of 2. Also a dense layer of 64.

The source code explanation of Conv2d is [here.](https://github.com/NervanaSystems/coach/blob/19ad2d60a7022bb5125855c029f27d86aaa46d64/rl_coach/architectures/tensorflow_components/layers.py)




#### Front facing camera

`"sensor": ["FRONT_FACING_CAMERA"]`
`"neural_network": "DEEP_CONVOLUTIONAL_NETWORK"`

This will create a network in the format:

Conv2d(32,5,2)

Conv2d(32,3,1)

Conv2d(64,3,2)

Conv2d(64,3,1)

dense_layer(64)


`"sensor": ["FRONT_FACING_CAMERA"]`
`"neural_network": "DEEP_CONVOLUTIONAL_NETWORK_SHALLOW"`

This will create a network in the format:

Conv2d(32,8,4)

Conv2d(64,4,2)

Conv2d(64,3,1)


`"sensor": ["FRONT_FACING_CAMERA"]`
`"neural_network": "DEEP_CONVOLUTIONAL_NETWORK_DEEP"`

This will create a network in the format:

Conv2d(32,8,4)

Conv2d(32,4,2)

Conv2d(64,4,2)

Conv2d(64,3,1)

dense_layer(512)

dense_layer(512)


#### Front facing camera with Lidar

`"sensor": ["FRONT_FACING_CAMERA", "LIDAR"]`
`"neural_network": "DEEP_CONVOLUTIONAL_NETWORK"`

This will create a network in the format:

Conv2d(32,5,2)

Conv2d(32,3,1)

Conv2d(64,3,2)

Conv2d(64,3,1)

dense_layer(64)

dense_layer(256)

dense_layer(256)


`"sensor": ["FRONT_FACING_CAMERA", "LIDAR"]`
`"neural_network": "DEEP_CONVOLUTIONAL_NETWORK_SHALLOW"`

This will create a network in the format:

Conv2d(32,8,4)

Conv2d(64,4,2)

Conv2d(64,3,1)

dense_layer(256)

dense_layer(256)


`"sensor": ["FRONT_FACING_CAMERA", "LIDAR"]`
`"neural_network": "DEEP_CONVOLUTIONAL_NETWORK_DEEP"`

This will create a network in the format:

Conv2d(32,8,4)

Conv2d(32,4,2)

Conv2d(64,4,2)

Conv2d(64,3,1)

dense_layer(512)

dense_layer(512)

dense_layer(256)

dense_layer(256)


#### Stereo Cameras

`"sensor": ["STEREO_CAMERAS"]`
`"neural_network": "DEEP_CONVOLUTIONAL_NETWORK"`

This will create a network in the format:

Conv2d(32,3,1)

Conv2d(64,3,2)

Conv2d(64,3,1)

Conv2d(128,3,2)

Conv2d(128,3,1)


`"sensor": ["STEREO_CAMERAS"]`
`"neural_network": "DEEP_CONVOLUTIONAL_NETWORK_SHALLOW"`

This will create a network in the format:

Conv2d(32,8,4)

Conv2d(64,4,2)

Conv2d(64,3,1)



`"sensor": ["STEREO_CAMERAS"]`
`"neural_network": "DEEP_CONVOLUTIONAL_NETWORK_DEEP"`

This will create a network in the format:

Conv2d(32,3,1)

Conv2d(64,3,2)

Conv2d(64,3,1)

Conv2d(128,3,2)

Conv2d(128,3,1)


#### Stereo Cameras with Lidar

`"sensor": ["STEREO_CAMERAS", "LIDAR"]`
`"neural_network": "DEEP_CONVOLUTIONAL_NETWORK"`

This will create a network in the format:

Conv2d(32,3,1)

Conv2d(64,3,2)

Conv2d(64,3,1)

Conv2d(128,3,2)

Conv2d(128,3,1)

dense_layer(256)

dense_layer(256)


`"sensor": ["STEREO_CAMERAS", "LIDAR"]`
`"neural_network": "DEEP_CONVOLUTIONAL_NETWORK_SHALLOW"`

This will create a network in the format:

Conv2d(32,8,4)

Conv2d(64,4,2)

Conv2d(64,3,1)

dense_layer(256)

dense_layer(256)


`"sensor": ["STEREO_CAMERAS", "LIDAR"]`
`"neural_network": "DEEP_CONVOLUTIONAL_NETWORK_DEEP"`

This will create a network in the format:

Conv2d(32,3,1)

Conv2d(64,3,2)

Conv2d(64,3,1)

Conv2d(128,3,2)

Conv2d(128,3,1)

dense_layer(256)

dense_layer(256)



### Save your action space artifact file and move to the next activity.



**[Proceed to the next activity](../modifyrewardfunction/)**
