# BeFine :  Real-time Distributed 3D Human Pose Estimation for Industrial Applications

There is an increasing interest in exploiting human pose estimation (HPE) software in human-machine interaction systems. Nevertheless, adopting such a computer vision application in real industrial scenarios is challenging. To overcome occlusion limitations, it requires multiple cameras, which in turn require multiple, distributed, and synchronized HPE software nodes running on resource-constrained edge devices. We address this challenge by presenting a real-time distributed 3D HPE platform, which consists of a set of 3D HPE software nodes on edge devices (i.e., one per camera) to redundantly extrapolate the human pose from different points of view.
A centralized aggregator collects the pose information through a shared communication network and merges them, in real time, through a pipeline of filtering, clustering and association algorithms. It addresses network communication issues (e.g., delay and bandwidth variability) through a two-levels synchronization, and supports both single and multi-person pose estimation. We present the evaluation results with a real-case of study (i.e., HPE for human-machine interaction in an intelligent manufacturing line), in which the platform accuracy and scalability are compared with state of the art approaches and with a marker-based infra-red motion capture system.



## Repository overview
This repository contains both the edge node code and the centralized aggregator code, respectively in *edge_node_mqtt* and *aggregator*. The first must be executed on an edge device, i.e., an Nvidia Jetson, the second on any server. The two devices must be connected to the same communication network. A mqtt broker (e.g. _mosquitto_) is required on a device in the network for message exchange.

## Requirements

The *edge* node requires:
* jetpack4.6.1 
* [trt_pose](https://github.com/NVIDIA-AI-IOT/trt_pose) 
* [json](https://github.com/nlohmann/json) 
* [cmake ](https://cmake.org) >= 3.12.2
* [torch](https://catalog.ngc.nvidia.com/orgs/nvidia/containers/pytorch) >= 1.7.0
* [torchvision](https://github.com/pytorch/vision)
* [paho mqtt](https://github.com/eclipse/paho.mqtt.cpp)
* [OpenCV](https://github.com/JetsonHacksNano/buildOpenCV.git) 
* [Stereolabs Zed SDK](https://www.stereolabs.com/developers/release/)
To install dependencies for the aggregator, on the centralized server:

```
cd BeFine/aggregator/
pip3 install -r requirements.txt
```

In addition edge nodes and the server aggregator need to be time synchronized with an NTP client (e.g. chrony) connected to a common NTP server (e.g. the server aggregator itself). In addition, the edge nodes take in input a configuration file with mqtt network configuration, topic, intrinsics and extrinsics matrix for common world reference. Finally, the server aggregator also need a configuration file with mqtt network configuration, input topics and aggregation settings.

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

