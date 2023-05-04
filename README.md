# BeFine :  Real-time Multi-camera 3D Human Pose Estimation for Industrial Applications

Human pose estimation (HPE) is a key component for an increasing number of human-cyber-physical systems. Nevertheless, applying such a computer vision technique in real industrial scenarios is a challenging task. Although solutions based on camera networks have shown great potential to overcome the occlusion limitations, real-time synchronization among multiple and networked cameras is still an open problem. Even more challenging is guaranteeing high accuracy when the HPE software has to be offloaded on resource-constrained embedded devices and the resulting data flows have to cross shared communication networks (e.g., Ethernet or WiFi) to be merged on a centralized unit. In this article we address this challenge by presenting a real-time HPE platform in which the 3D poses are estimated through a network of edge computing systems. A centralized aggregator collects the information through MQTT and merges them, in real time, through a pipeline of filtering, clustering and association algorithms. It addresses network communication issues (e.g., delay and bandwidth variability) through a two-levels synchronization, and supports both single and multi-person pose estimation. The article presents the results with a real-case of study (i.e., HPE for human-machine interaction in an intelligent manufacturing line), in which the platform accuracy and scalability are compared with state of the art approaches and with a marker-based infra-red motion capture system.



## Repository overview
This repository contains both the edge node code and the centralized aggregator code, respectively in *edge_node_mqtt* and *aggregator*. The first must be executed on an edge device, i.e., an Nvidia Jetson, the second on any server. The two devices must be connected to the same communication network. A mqtt broker (e.g. _mosquitto_) is required on a device in the network for message exchange.

## Requirements

The *edge* node requires:
* jetpack4.6.1 
* trtpose 
* json 
* cmake 
* torch 
* torchvision
* paho mqtt

To install dependencies for the aggregator, on the centralized server:

```
cd BeFine/aggregator/
pip3 install -r requirements.txt
```

## Execute the demo

On the server:

```
cd aggregator/

python3 multip_aggregator.py
```

On the edge node:

```
cd edge_node_mqtt/
mkdir build
cd build
cmake ..
make -j
./pub_kp3d 
```

