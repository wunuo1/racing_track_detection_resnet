# Function Introduction

Based on deep learning methods, identify the midpoint of the track in the image and publish a message using the ResNet18 model

# Usage

## Preparations

Equipped with a real robot or robot simulation module, including a motion chassis, camera, and RDK kit, and able to operate normally.

## Compile and Run

**1.Compile**

After starting the robot, connect to it through SSH or VNC on the terminal, click the "One click Deployment" button in the upper right corner of this page, copy the following command and run it on the RDK system to complete the installation of the relevant nodes.

```bash
# Pull the line center detection code code
mkdir -p ~/racing_ws/src && cd ~/racing_ws/src

git clone https://github.com/wunuo1/racing_track_detection_resnet.git -b feature-x3

# Compile
cd ..

# humble
source /opt/tros/humble/setup.bash
# foxy
source /opt/tros/setup.bash

colcon build
```

**2.Operate the line patrol perception function**

```shell
source ~/racing_ws/install/setup.bash
cp -r ~/racing_ws/install/racing_track_detection_resnet/lib/racing_track_detection_resnet/config/ .

# Web side visualization track midpoint (open IP: 8000 in the browser after starting the function)

export WEB_SHOW=TRUE

# Simulation (using simulation models)
ros2 launch racing_track_detection_resnet racing_track_detection_resnet_simulation.launch.py

# Actual scenario (using models from actual scenarios)
ros2 launch racing_track_detection_resnet racing_track_detection_resnet.launch.py
```


# Principle Overview

RDK obtains environmental data in front of the car through a camera, and the image data is inferred through a trained CNN model to obtain the coordinate values of the guide line and published.

# Interface Description

## Topics

### Published Topics

| Name                          | Type                        | Description                      |
| ----------------------------- | ------------------------------------------------------------ | ------------------------------------------------------ |
| /racing_track_center_detection                      | ai_msgs::msg::PerceptionTargets               | Publish the image coordinates of the midpoint of the track                 |

### Subscribed Topics
| Name                          | Type                       | Description                       |
| ----------------------------- | ------------------------------------------------------------ | ------------------------------------------------------ |
| /hbmem_img                     | hbm_img_msgs/msg/HbmMsg1080P                                    | Receive image messages posted by the camera (640x480)                   |

## Parameters

| Parameter Name        | Type        | Description   |
| --------------------- | ----------- | -------------------------------------------------------------------------------------------------- |
| model_path    | string | The model file used for inference should be configured according to the actual model path. The default value is config/race_track_detection_simulation.bin |
| sub_img_topic | string |  Please configure the received image topic name based on the actual received topic name. The default value is /hbmem_img |

# Note
This feature pack provides models that can be used in the Gazebo simulation environment as well as models that can be used in specific real-world scenarios. If you collect your own dataset for training, please note to replace it.